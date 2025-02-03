#include "demon_core/demon_core.hpp"
#include "record_log.hpp"

/**
* @brief  构造函数
* @param  config_path：配置文件地址
* @return 无
*/
Demon_Core_t::Demon_Core_t(){
    if(!parm.readParam()) return;

    isOpen = true;

    // mavlink初始化
    if(parm.if_launch_mavlink)
    {
        if(mav_msg.init(parm.fc_serialPath, parm.udp_address)){
            std::cout << "\033[33m" << "*************** [Demon_Core_t] Mavlink_Manager_t初始化完成 ***************" << "\033[0m" << std::endl << std::endl;
            if_mavlink_ready = true;
            get_imu_thread();   // 获取mavlink消息-->imu
            get_gps_thread();   // 获取mavlink消息-->GPS
            get_rc_thread();    // 获取mavlink消息-->RC
            usleep(200*1000);
        }
        else std::cout << "\033[33m" << "*************** [Demon_Core_t] Mavlink_Manager_t初始化失败 ***************" << "\033[0m" << std::endl << std::endl;
    }

    // 双目初始化
    if(stereoMatch.init(parm.config_folder + "/stereo_config.yaml")){
        std::cout << "\033[33m" << "*************** [Demon_Core_t] stereoMatch初始化完成 ***************" << "\033[0m" << std::endl << std::endl; 
        if_stereo_ready = true;
        get_img_thread();       // 获取左右原始图像-->发给vins、发布ros
        get_depth_Pose_thread();// 获取深度图像--> 从视差图变为深度图
        get_img_rect_thread();  // 获取校正图像--> 进行行人识别
    }
    else
        std::cout << "\033[31m" << "*************** [Demon_Core_t] stereoMatch初始化失败 ***************" << "\033[0m" << std::endl << std::endl; 

    // vins初始化。需要mavlink和stereo都准备好了
    if(parm.if_launch_vins)
    {
        if(if_mavlink_ready && if_stereo_ready && readParameters(parm.config_folder + "/stereo_imu_config.yaml")){
            estimator.setParameter();
            std::cout << "\033[33m" << "*************** [Demon_Core_t] VINS初始化完成 ***************" << "\033[0m" << std::endl << std::endl; 
            if_vins_ready = true;
            get_odom_thread();      // 获取里程计--> 坐标系转换、发给mavlink、发布ros、发给planner
        }
    }

    // 路径规划初始化
    if(parm.if_launch_planner)
    {
        if(if_vins_ready && adaptive_replan.init(parm.config_folder + "/SGP_Planner.yaml")){
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::cout << "\033[33m" << "*************** [Demon_Core_t] SGP_Planner初始化完成 ***************" << "\033[0m" << std::endl << std::endl;
            if_planner_ready = true;
            get_navigation_thread();// 获取导航线程 
            visualization_thread();    
        }
        else
            std::cout << "\033[31m" << "*************** [Demon_Core_t] SGP_Planner初始化失败 ***************" << "\033[0m" << std::endl << std::endl; 
    }

    if(if_planner_ready && controller.init(parm.config_folder + "/so3_config.yaml"))
    {
        std::cout << "\033[33m" << "*************** [Demon_Core_t] so3_controller初始化完成 ***************" << "\033[0m" << std::endl << std::endl;
        if_so3_ready = true;
        so3_controll_thread();
    }

    // 目标识别初始化
    if(parm.if_launch_people_detect && people_detect.init(std::string(getenv("HOME"))+"/demon_drone_ws/src/demon_core/models")){
        if_people_detect_ready = true;
        std::cout << "\033[33m" << "*************** [Demon_Core_t] people_detect初始化完成 ***************" << "\033[0m" << std::endl << std::endl; 
    }

    std::cout << "\033[32m" << "[Demon_Core_t] demon_core_node 启动！" << "\033[0m" << std::endl << std::endl;   
    
    // if(if_planner_ready) adaptive_replan.setGoal(Eigen::Vector3d(10, 0, 1.5));
}

Demon_Core_t::~Demon_Core_t(void){
    isOpen = false;
    std::cout << "\033[35m" << "[Demon_Core_t]\t\t 析构成功，Demon_Core已经关闭！" << "\033[0m" << std::endl;
}

void Demon_Core_t::spin(int argc, char** argv){
    if(parm.if_use_ros)
    {
        ros.spin(argc, argv, true, false, false);
    }
    else usleep(10000*1000);
}

/**
* @brief  获取图像线程。将图像原始数据压入vins，并发布为ros话题
* @param  无
* @return 无
*/
void Demon_Core_t::get_img_thread(void){
    std::thread ThRecv	= std::thread{
        [&](){
            std::cout << "\033[32m" << "[Demon_Core_t] 获取左右原始图像线程开始......" << "\033[0m" << std::endl;
            while (isOpen){
                stereoMatch.imgRaw_ready.lock();
                // 获取左右原始图像
                double img_t = stereoMatch.lFrame.seconds;
                #ifdef USE_ROS1
                    if(ros.ready){
                        img_t = ros::Time::now().toSec() - getSteadySec() + img_t;
                        // std::cout << fixed << ros::Time::now().toSec() << std::endl;
                    }
                #else

                #endif

                // vins输入图像
                cv::Mat lFrame, rFrame;
                cv::cvtColor(stereoMatch.lFrame.data, lFrame, cv::COLOR_BGR2GRAY);
                cv::cvtColor(stereoMatch.rFrame.data, rFrame, cv::COLOR_BGR2GRAY);

                if(if_vins_ready) estimator.inputImage(img_t, lFrame, rFrame);
                // 发布图像
                ros.pub_Image(img_t, stereoMatch.lFrame.data, stereoMatch.rFrame.data);
                std::cout << ros::ok() << std::endl;
            }
            std::cout << "\033[34m" << "[Demon_Core_t] 结束获取左右原始图像线程！" << "\033[0m" << std::endl;
        }
    };
    ThRecv.detach();
}

void Demon_Core_t::get_img_rect_thread(void){
    std::thread ThRecv	= std::thread{
        [&](){
            std::cout << "\033[32m" << "[Demon_Core_t] 获取校正图像线程开始......" << "\033[0m" << std::endl;
            while (isOpen){
                stereoMatch.imgRect_ready.lock();
                double tNow = getSteadySec();
                cv::Mat lFrame_rec = stereoMatch.lFrame_rec.data.clone();
                if(if_people_detect_ready){
                    people_detect.process(lFrame_rec);
                    people_box_que_mutex.lock();
                    if(people_detect.people_boxs.size() > 0){
                        people_box_que.push(people_detect.people_boxs);
                        people_box_t_que.push(stereoMatch.lFrame_rec.seconds);
                    }
                    if(people_box_que.size() > 3) people_box_que.pop();
                    if(people_box_t_que.size() > 3) people_box_t_que.pop();
                    people_box_que_mutex.unlock();

                    if(parm.if_record_video || parm.if_show_people_detect){
                        char text[256];
                        for (int i = 0; i < (int)people_detect.people_boxs.size(); i++) {
                            auto it = peoples_pos.find((int)people_detect.people_boxs[i].trackID);
                            if(it != peoples_pos.end()){
                                sprintf(text, "%s %.1f%% %d :%.1f,%.1f,%.1f", people_detect.people_boxs[i].name.c_str()
                                                            , people_detect.people_boxs[i].confidence * 100
                                                            , (int)people_detect.people_boxs[i].trackID
                                                            , peoples_pos[(int)people_detect.people_boxs[i].trackID][0]
                                                            , peoples_pos[(int)people_detect.people_boxs[i].trackID][1]
                                                            , peoples_pos[(int)people_detect.people_boxs[i].trackID][2]);
                                // printf("id=%d, x=%.1f, y=%.1f, z=%.1f\n", (int)people_detect.people_boxs[i].trackID
                                //                                       ,peoples_pos[(int)people_detect.people_boxs[i].trackID][0]
                                //                                       ,peoples_pos[(int)people_detect.people_boxs[i].trackID][1]
                                //                                       ,peoples_pos[(int)people_detect.people_boxs[i].trackID][2]);
                            }
                            else
                                sprintf(text, "%s %.1f%% %d", people_detect.people_boxs[i].name.c_str()
                                                            , people_detect.people_boxs[i].confidence * 100
                                                            , (int)people_detect.people_boxs[i].trackID);
                            int x1 = people_detect.people_boxs[i].x1;
                            int y1 = people_detect.people_boxs[i].y1;
                            int x2 = people_detect.people_boxs[i].x2;
                            int y2 = people_detect.people_boxs[i].y2;
                            // printf("%s @ (%d %d %d %d) %f\n",people_detect.people_boxs[i].name.c_str(), x1, y1, x2, y2, people_detect.people_boxs[i].confidence);

                            if((int)people_detect.people_boxs[i].trackID == lock_people_id)
                                rectangle(lFrame_rec, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255, 255), 3.0);
                            else
                                rectangle(lFrame_rec, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0, 255), 2.0);
                            putText(lFrame_rec, text, cv::Point(x1, y1 + 12), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));  // BGR
                        }

                        // 计算耗时和帧率
                        static double last_t = 0;
                        static double fps = 10.0;
                        static int cost_t = 100;
                        if(last_t == 0) last_t = tNow;
                        else{
                            fps = 0.97 * fps + 0.03 * 1.0 / (tNow - last_t); // 
                            last_t = tNow;
                        }
                        cost_t = 0.97 * cost_t + 0.03 * (getSteadySec() - tNow)*1000;
                        cv::putText(lFrame_rec, "time: " + std::to_string(cost_t) + "[ms] " // 
                                                    + std::to_string(fps).substr(0, std::to_string(fps).find(".") + 1 + 1) + "[FPS]",
                                                    cv::Point(0, 20), 2, 0.75, cv::Scalar(255, 255, 255));
                        if(parm.if_show_people_detect){
                            cv::imshow("yolov5", lFrame_rec);
                            int key = cv::waitKey(1);
                            if (27 == key || 'q' == key || 'Q' == key) exit(1);                        
                        }
                    }
                }
                else{
                    // 发布校正图像
                }
                if(parm.if_record_video) record_video(lFrame_rec);
            }
            std::cout << "\033[34m" << "[Demon_Core_t] 结束获取校正图像线程！" << "\033[0m" << std::endl;
        }
    };
    ThRecv.detach();
}

/**
* @brief  获取mavlink_imu数据线程。将imu数据压入vins，并发布为ros话题
* @param  无
* @return 无
*/
void Demon_Core_t::get_imu_thread(void){
    std::thread ThRecv	= std::thread{
        [&](){
            std::cout << "\033[32m" << "[Demon_Core_t] 获取imu数据线程开始......" << "\033[0m" << std::endl;
            while(isOpen && mav_msg.isOpen){
                if(mav_msg.imu_data_ready){
                    mav_msg.imu_data_ready = false;
                    // 获取imu数据
                    double imu_t = mav_msg.imu_t;
                    #ifdef USE_ROS1
                        if(ros.ready){
                            imu_t = ros::Time::now().toSec() - getSteadySec() + imu_t;
                        }
                    #else

                    #endif
                    Eigen::Vector3d acc(mav_msg.highres_imu_data.xacc , -mav_msg.highres_imu_data.yacc , -mav_msg.highres_imu_data.zacc );
                    Eigen::Vector3d gyr(mav_msg.highres_imu_data.xgyro, -mav_msg.highres_imu_data.ygyro, -mav_msg.highres_imu_data.zgyro);
                    mag = Eigen::Vector3d(mav_msg.highres_imu_data.xmag,-mav_msg.highres_imu_data.ymag , -mav_msg.highres_imu_data.zmag );

                    if(if_vins_ready)   estimator.inputIMU(imu_t, acc, gyr);    // vins的imu消息压入
                    if(if_so3_ready)    controller.estimateThrustModel(acc);
                    ros.pub_imu(imu_t, acc, gyr);   // imu数据发布
                    // 测试帧率
                    double tNow = getSteadySec();
                    static double last_t = 0;
                    static double fps = 100;
                    static int count = 0;
                    if(last_t == 0) last_t = tNow;
                    else{
                        double delta_t = tNow - last_t;
                        count ++;
                        if(delta_t >= 1.0){
                            fps = count / delta_t;
                            last_t = tNow;
                            count = 0;
                            std::cout << fps << std::endl;
                        } 
                    }
                }
                else usleep(2*1000);
            }
            std::cout << "\033[34m" << "[Demon_Core_t] 获取imu数据线程结束！" << "\033[0m" << std::endl;
        }
    };
    ThRecv.detach();
}

/**
* @brief  获取mavlink_gps数据线程。将imu数据压入，并发布为ros话题
* @param  无
* @return 无
*/
void Demon_Core_t::get_gps_thread(void){
    std::thread ThRecv	= std::thread{
        [&](){
            std::cout << "\033[32m" << "[Demon_Core_t] 获取GPS数据线程开始......" << "\033[0m" << std::endl;
            while(isOpen){
                mav_msg.gps_ready.lock();
                double gps_t = mav_msg.gps_t;
                double latitude  = mav_msg.gps_data.lat / 10000000.0;
                double longitude = mav_msg.gps_data.lon / 10000000.0;
                double altitude  = mav_msg.gps_data.alt / 1000.0;
                int satellites_visible = mav_msg.gps_data.satellites_visible;
                ros.pub_gps(gps_t, latitude, longitude, altitude, satellites_visible);

                if(mav_msg.gps_data.satellites_visible < 10) continue;
                // std::cout << "\033[32m" << "卫星数：" << mav_msg.gps_data.satellites_visible << "\033[0m" << std::endl;
                gps_que_mutex.lock();
                gps_que.push(std::vector<double>{gps_t, latitude, longitude, altitude, (double)satellites_visible});
                if(gps_que.size() > 4)  gps_que.pop();
                gps_que_mutex.unlock();
            }
            std::cout << "\033[34m" << "[Demon_Core_t] 获取GPS数据线程结束！" << "\033[0m" << std::endl;
        }
    };
    ThRecv.detach();
}

/**
* @brief  获取mavlink_rc数据
* @param  无
* @return 无
*/
void Demon_Core_t::get_rc_thread(void){
    std::thread ThRecv	= std::thread{
        [&](){
            std::cout << "\033[32m" << "[Demon_Core_t] 获取RC数据线程开始......" << "\033[0m" << std::endl;
            while(isOpen){
                mav_msg.rc_ready.lock();
                if(mav_msg.rc_data.port < 8) continue;
                rc.SWA = mav_msg.rc_data.chan5_raw < 900 ? 1 : 2;
                if(mav_msg.rc_data.chan6_raw < 500)         rc.SWB = 1;
                else if(mav_msg.rc_data.chan6_raw < 1500)   rc.SWB = 2;
                else                                        rc.SWB = 3;
                if(mav_msg.rc_data.chan7_raw < 500)         rc.SWC = 1;
                else if(mav_msg.rc_data.chan7_raw < 1500)   rc.SWC = 2;
                else                                        rc.SWC = 3;

                rc.SWD = mav_msg.rc_data.chan8_raw < 900 ? 1 : 2;

                // rc.mode = NONE_MODE;
                if(rc.SWD == 1)                 rc.mode = MANUAL_MODE;
                else if(rc.SWD == 2){
                    if(rc.SWC == 2)             rc.mode = OFFBOARD_MANUAL_MODE;
                    else if(rc.SWC == 3){
                        if(rc.SWB == 1)         rc.mode = OFFBOARD_NAVI_MODE;
                        else if(rc.SWB == 3)    rc.mode = OFFBOARD_TEST_MODE;
                    }
                }
                
                if(rc.mode != rc.last_mode){    // 判断mode是否更新
                    std::cout << "\033[34m" << "[Demon_Core_t] 退出模式  ：" << mode_to_string(rc.last_mode) << "\033[0m" << std::endl;
                    std::cout << "\033[32m" << "[Demon_Core_t] 切换到模式：" << mode_to_string(rc.mode) << "\033[0m" << std::endl;
                    if(rc.last_mode == OFFBOARD_NAVI_MODE || rc.last_mode == OFFBOARD_TEST_MODE){
                        if(if_planner_ready) adaptive_replan.resetState();
                    }
                    if(rc.mode == OFFBOARD_TEST_MODE){
                        if(if_planner_ready) adaptive_replan.setGoal(Eigen::Vector3d(10, 0, 1.5));
                    }
                    rc.last_mode = rc.mode;
                }
            }
            std::cout << "\033[34m" << "[Demon_Core_t] 获取RC数据线程结束！" << "\033[0m" << std::endl;
        }
    };
    ThRecv.detach();
}


/**
* @brief  获取VINS里程计的线程。机体里程计发送给mavlink、相机位置加入队列、发送ros话题
* @param  无
* @return 无
*/
void Demon_Core_t::get_odom_thread(void){
    std::thread ThRecv	= std::thread{
        [&](){
            // bool if_init = false;
            // Eigen::Isometry3d T_mv = Eigen::Isometry3d::Identity(); // 地磁坐标系下的vio位置
            Eigen::Isometry3d T_vi = Eigen::Isometry3d::Identity(); // vio坐标系下的imu位置
            Eigen::Isometry3d T_vb = Eigen::Isometry3d::Identity(); // vio坐标系下的body位置
            Eigen::Isometry3d T_vc = Eigen::Isometry3d::Identity(); // vio坐标系下的cam位置
            Eigen::Isometry3d T_gb = Eigen::Isometry3d::Identity(); // gps坐标系下的body位置
            Eigen::Isometry3d T_gc = Eigen::Isometry3d::Identity(); // gps坐标系下的cam位置
            int satellites_visible = 0;
            std::cout << "\033[32m" << "[Demon_Core_t] 获取里程计线程开始......" << "\033[0m" << std::endl;
            while(isOpen){
                estimator.odom_ready.lock();
                double img_t = estimator.img_t;
                // VIO系下imu的位置（VINS输出）
                T_vi = Eigen::Isometry3d::Identity();
                T_vi.rotate(estimator.Rs[WINDOW_SIZE]);
                T_vi.pretranslate(estimator.Ps[WINDOW_SIZE]);
                Eigen::Vector3d vel = estimator.Vs[WINDOW_SIZE];

                // if(parm.if_use_gps && !if_init){
                //     if_init = true;
                //     Eigen::Vector3d euler = get_euler_from_matrix(T_vi.rotation());
                //     double mag_yaw = atan2(-mag[0]*cos(euler[1])+mag[2]*sin(euler[1]),
                //                             (mag[1]*cos(euler[0]) + mag[0]*sin(euler[0])*sin(euler[1]) + mag[2]*sin(euler[0])*cos(euler[1])));
                //     std::cout << "\033[32m" << "[Demon_Core_t] 磁力计初始化方向为：" << mag_yaw/M_PI*180 << "\033[0m" << std::endl;
                //     Eigen::Matrix3d rotation_matrix1;
                //     rotation_matrix1 =  Eigen::AngleAxisd(mag_yaw-M_PI/2.0, Eigen::Vector3d::UnitZ()) *
                //                         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                //                         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
                //     T_mv.rotate(rotation_matrix1);
                //     T_mv.pretranslate(Eigen::Vector3d(0, 0, 0));
                // }

                /* tf转换，获取VIO坐标系下的body和camera */
                T_vb = T_vi * parm.T_ib;
                T_vc = T_vi * parm.T_ic;

                Pos_Msg_t body_pos_msg, cam_pos_msg;
                Eigen::Vector3d gps_pos, global_pos;
                Eigen::Vector3d vb_pos      = T_vb.translation();
                Eigen::Quaterniond vb_quad  = Eigen::Quaterniond(T_vb.rotation());
                bool if_gps_update = false;
                
                if(gps_que.size() > 0){  // 使用GPS
                    // 时间戳对齐
                    gps_que_mutex.lock();
                    while(gps_que.size() > 0){
                        std::vector<double> gps_data = gps_que.front();
                        gps_que.pop();
                        if(gps_que.size() == 0 || abs(img_t - gps_data[0]) < abs(img_t -  gps_que.front()[0])){
                           globalEstimator.GPS2XYZ(gps_data[1], gps_data[2], gps_data[3], gps_pos); // 获取GPS的xyz位置
                           satellites_visible = gps_data[4];
                           break;
                        }
                    }
                    gps_que_mutex.unlock();
                    globalEstimator.inputOdom(img_t, vb_pos, vb_quad);  // 输入VIO坐标系下body位置
                    globalEstimator.inputGPS(img_t, gps_pos, 1);
                    T_gb = globalEstimator.WGPS_T_WVIO * T_vb;
                    T_gc = globalEstimator.WGPS_T_WVIO * T_vc;
                    global_pos = T_gb.translation();
                    if_gps_update = true;
                }

                if(parm.if_use_gps && if_gps_update){
                    /* 获取GPS坐标系下机体位置 */
                    body_pos_msg.t      = img_t;
                    body_pos_msg.pos    = T_gb.translation();
                    body_pos_msg.quad   = Eigen::Quaterniond(T_gb.rotation());
                    body_pos_msg.euler  = get_euler_from_matrix(T_gb.rotation());
                    /* 获取GPS坐标系下相机位置 */
                    cam_pos_msg.t       = img_t;
                    cam_pos_msg.pos     = T_gc.translation();
                    cam_pos_msg.quad    = Eigen::Quaterniond(T_gc.rotation());                    
                    cam_pos_msg.euler   = get_euler_from_matrix(T_gc.rotation());
                }
                else{
                    /* 获取GPS坐标系下机体位置 */
                    body_pos_msg.t      = img_t;
                    body_pos_msg.pos    = T_vb.translation();
                    body_pos_msg.quad   = Eigen::Quaterniond(T_vb.rotation());
                    body_pos_msg.euler  = get_euler_from_matrix(T_vb.rotation());
                    /* 获取GPS坐标系下相机位置 */
                    cam_pos_msg.t       = img_t;
                    cam_pos_msg.pos     = T_vc.translation();
                    cam_pos_msg.quad    = Eigen::Quaterniond(T_vc.rotation());                    
                    cam_pos_msg.euler   = get_euler_from_matrix(T_vc.rotation());                    
                }

                /* 相机位置压入队列 */
                camPos_que_mutex.lock();
                camPos_que.push(cam_pos_msg);
                if(camPos_que.size() > 4) camPos_que.pop();
                camPos_que_mutex.unlock();

                if(if_planner_ready)    adaptive_replan.setOdometry(body_pos_msg.pos, vel, body_pos_msg.quad);  // 设置路径规划里程计
                if(if_so3_ready)
                {
                    controller.SetOdometry(img_t, body_pos_msg.pos, vel, body_pos_msg.quad);// 设置线性控制器里程计
                    if(rc.mode < OFFBOARD_NAVI_MODE)    controller.SetTargetState(body_pos_msg.pos, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0), body_pos_msg.quad);
                }
                
                mav_msg.send_pos_to_mavlink(img_t, body_pos_msg.pos, body_pos_msg.euler);
                mav_msg.send_vel_to_mavlink(img_t, vel);
                // usleep(8*1000);
                record_vb_odom(img_t, vb_pos, vel, vb_quad);
                if(if_gps_update){
                    record_gb_pos(img_t, body_pos_msg.pos, body_pos_msg.quad);
                    record_GPS_pos(img_t, gps_pos); // 记录GPS位置
                }
                ros.pub_odom(img_t, body_pos_msg.pos, vel, body_pos_msg.quad);
                ros.drawDroneTraj(img_t, body_pos_msg.pos, vel);
                if(parm.if_show_info){
                    //固定的浮点显示
                    std::cout.setf(std::ios::fixed);
                    std::cout << std::setprecision(2); //设显示小数精度为n位
                    //左对齐
                    std::cout.setf(std::ios::left);
                    // 强制显示小数点
                    std::cout.setf(std::ios::showpoint);
                    // 强制显示符号
                    std::cout.setf(std::ios::showpos);

                    std::cout <<">>>>>>>>>>>>>>>>>>>>>>>> Drone Info [ENU Frame] <<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
                    std::cout << "\033[32m" << "当前模式为  : " << mode_to_string(rc.mode) << "\033[0m" << std::endl;
                    std::cout << "GPS_卫星数量: " << satellites_visible << std::endl;
                    std::cout << "Drone_pose_g [x y z]: " << global_pos[0] << " [m] " << global_pos[1] << " [m] " << global_pos[2] << " [m] " << std::endl;
                    std::cout << "Drone_pose_v [x y z]: " << vb_pos[0] << " [m] " << vb_pos[1] << " [m] " << vb_pos[2] << " [m] " << std::endl;
                    std::cout << "Drone_vel    [x y z]: " << vel[0] << " [m/s] " << vel[1] << " [m/s] " << vel[2] << " [m/s] " << std::endl;
                    std::cout << "Drone_euler  [R P Y]: " << body_pos_msg.euler[0]/M_PI*180.0 << " [deg] " << body_pos_msg.euler[1]/M_PI*180.0 << " [deg] " << body_pos_msg.euler[2]/M_PI*180.0 << " [deg] " << std::endl;
                    std::cout <<">>>>>>>>>>>>>>>>>>>>>>>> Camer Info [ENU Frame] <<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
                    std::cout << "camer_pose_w [x y z]: " << cam_pos_msg.pos[0] << " [m] " << cam_pos_msg.pos[1] << " [m] " << cam_pos_msg.pos[2] << " [m] " << std::endl;
                    std::cout << "camer_euler  [R P Y]: " << cam_pos_msg.euler[0]/M_PI*180.0 << " [deg] " << cam_pos_msg.euler[1]/M_PI*180.0 << " [deg] " << cam_pos_msg.euler[2]/M_PI*180.0 << " [deg] " << std::endl;                
                    std::cout << std::endl;
                }  
            }
            std::cout << "\033[34m" << "[Demon_Core_t] 获取里程计线程结束！" << "\033[0m" << std::endl;
        }
    };
    ThRecv.detach();
}

/**
* @brief  获取导航线程
*/
void Demon_Core_t::get_navigation_thread(void){
    std::thread ThRecv	= std::thread{
        [&](){
            std::cout << "\033[32m" << "[Demon_Core_t] 获取导航线程开始......" << "\033[0m" << std::endl;
            while(isOpen){
                adaptive_replan.ctrCmd_ready.lock();
                CtrMsg_t ctrCmd = adaptive_replan.ctrCmd;
                if(if_so3_ready) controller.SetTargetState(ctrCmd.pos, ctrCmd.vel, ctrCmd.acc, ctrCmd.yaw);
                // mav_msg.send_setpoint_to_mavlink(ctrCmd.pos, ctrCmd.vel, ctrCmd.acc, ctrCmd.yaw, ctrCmd.yaw_dot);
                // record_waypoint(ctrCmd.stamp_t, ctrCmd.pos);
                // ros.drawCmd(ctrCmd.stamp_t, ctrCmd.pos, ctrCmd.vel, 0, Eigen::Vector4d(0, 1, 0, 1));
                // ros.drawCmd(ctrCmd.stamp_t, ctrCmd.pos, ctrCmd.acc, 1, Eigen::Vector4d(0, 0, 1, 1));
                // Eigen::Vector3d dir(cos(ctrCmd.yaw), sin(ctrCmd.yaw), 0.0);
                // ros.drawCmd(ctrCmd.stamp_t, ctrCmd.pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));                    
            }
            std::cout << "\033[34m" << "[Demon_Core_t] 获取导航线程结束！" << "\033[0m" << std::endl;
        }
    };
    ThRecv.detach();
}

/**
* @brief  线性控制器线程
*/
void Demon_Core_t::so3_controll_thread(void)
{
    std::thread ThRecv	= std::thread{
        [&](){
            std::cout << "\033[32m" << "[Demon_Core_t] so3控制器线程开始......" << "\033[0m" << std::endl;
            while(isOpen){
                Controller_Output_t u = controller.calculateControl();
                Eigen::Matrix3d R = u.q.toRotationMatrix();
                Eigen::Vector3d euler_angles = get_euler_from_matrix(R); //  R.eulerAngles(0, 1, 2); 
                std::cout << "目标角度(RPY)：" << euler_angles[0]/M_PI*180.0 << " [deg] " << euler_angles[1]/M_PI*180.0 << " [deg] " << euler_angles[2]/M_PI*180.0 << " [deg] " << u.thrust << "推力" << std::endl; 

                if(rc.mode >= OFFBOARD_NAVI_MODE) 
                    mav_msg.send_AttitudeTarget_to_mavlink(u.q, u.thrust);
                usleep(10);          
            }
            std::cout << "\033[34m" << "[Demon_Core_t] so3控制器线程结束！" << "\033[0m" << std::endl;
        }
    };
    ThRecv.detach();
}


/**
* @brief  获取深度图和相机位置线程。需要进行时间戳的同步处理
* @param  无
* @return 无
*/
void Demon_Core_t::get_depth_Pose_thread(void){
    std::thread ThRecv	= std::thread{
        [&](){
            std::cout << "\033[32m" << "[Demon_Core_t] 获取深度图线程开始......" << "\033[0m" << std::endl;
            while(isOpen){
                stereoMatch.depth_ready.lock();
                double depth_t = stereoMatch.depth_img.seconds;
                #ifdef USE_ROS1
                    if(ros.ready){
                        depth_t = ros::Time::now().toSec() - getSteadySec() + depth_t;
                        // std::cout << fixed << ros::Time::now().toSec() << std::endl;
                    }
                #else

                #endif
                ros.pub_depth(depth_t, stereoMatch.depth_img.data);

                if(if_vins_ready)
                {
                    /***** 进行深度图像和相机位置时间戳同步，并发给路径规划 *****/
                    camPos_que_mutex.lock();
                    while(camPos_que.size() > 0){
                        if(abs(camPos_que.front().t - depth_t) < 0.02){
                            // std::cout << "深度图及相机位置已经设置" << std::endl;
                            if(if_planner_ready)
                                adaptive_replan.planner_manager_->sdf_map_->setDepthPose(stereoMatch.depth_img.data, camPos_que.front().pos, camPos_que.front().quad);
                            ros.pub_cam_pos(depth_t, camPos_que.front().pos, camPos_que.front().quad);
                            // ros.pub_depth(depth_t, stereoMatch.depth_img.data);
                            camPos_que.pop();                     
                            break;
                        }
                        else camPos_que.pop();
                    }
                    camPos_que_mutex.unlock();
                }

                if(if_people_detect_ready)
                {
                    /***** 进行深度图像和AI模型识别结果时间戳同步 *****/
                    people_box_que_mutex.lock();
                    while(people_box_que.size() > 0){
                        if(abs(people_box_t_que.front() - depth_t) < 0.02){
                            for(int i=0; i<(int)people_box_que.front().size(); i++){
                                int people_id = people_box_que.front()[i].trackID;
                                int u = (people_box_que.front()[i].x1 + people_box_que.front()[i].x2)/2;    // 像素坐标, 宽
                                int v = (people_box_que.front()[i].y1 + people_box_que.front()[i].y2)/2;    // 像素坐标, 高
                                // std::cout << "people_id=" << people_id << ",u=" << u << ",v=" << v << std::endl;
                                peoples_pos[people_id][2] = stereoMatch.depth_img.data.at<uint16_t>(v,u)/1000.0;                        // z
                                peoples_pos[people_id][0] = (u - stereoMatch.depth_u0)*peoples_pos[people_id][2]/stereoMatch.depth_fxy; // x
                                peoples_pos[people_id][1] = (v - stereoMatch.depth_v0)*peoples_pos[people_id][2]/stereoMatch.depth_fxy; // y
                            }
                            people_box_que.pop();
                            people_box_t_que.pop();               
                            break;
                        }
                        else{
                            // printf("ai_t=%.3f, depth_t=%.3f\n", people_box_t_que.front(), depth_t);
                            people_box_que.pop();
                            people_box_t_que.pop();   
                        }
                        // std::cout << std::endl;
                    }
                    people_box_que_mutex.unlock();
                }
            }
            std::cout << "\033[34m" << "[Demon_Core_t] 获取深度图线程结束！" << "\033[0m" << std::endl;
        }
    };
    ThRecv.detach();
}

void Demon_Core_t::visualization_thread(void){
    std::thread ThRecv	= std::thread{
        [&](){
            std::cout << "\033[32m" << "[Demon_Core_t] 可视化线程开始......" << "\033[0m" << std::endl;
            while(isOpen){
                double now_t = getSteadySec();
                // 显示点云
                if(if_planner_ready){
                    ros.set_origin_pos(adaptive_replan.origin_pos);

                    if(adaptive_replan.have_goal_){
                        ros.drawLocalGoal(now_t, adaptive_replan.goal_pos, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
                    }
                    if(adaptive_replan.planner_manager_->sdf_map_->if_cloud_ready){
                        adaptive_replan.planner_manager_->sdf_map_->if_cloud_ready = false;
                        ros.pub_point_cloud(adaptive_replan.planner_manager_->sdf_map_->occpy_cloud);
                    }
                    if(adaptive_replan.if_low_mpc_traj_ready){
                        adaptive_replan.if_low_mpc_traj_ready = false;
                        ros.drawAstar(now_t, adaptive_replan.planner_manager_->local_path_, 0.08, Eigen::Vector4d(0, 0, 1, 1.0));
                        ros.drawLowMpcTraj(now_t, adaptive_replan.planner_manager_->low_mpc_traj_, 0.1, Eigen::Vector4d(0, 1, 0, 1.0));
                    }
                    if(adaptive_replan.if_high_mpc_traj_ready){
                        adaptive_replan.if_high_mpc_traj_ready = false;
                        ros.drawHighMpccTraj(now_t, adaptive_replan.planner_manager_->high_mpcc_traj_pos_, 0.2, Eigen::Vector4d(1, 0, 0, 1.0));
                        // ros.drawHighMpccRefTraj(now_t, adaptive_replan.planner_manager_->high_mpcc_traj_ref_, 0.1, Eigen::Vector4d(0, 0, 1, 1.0));
                    }
                }
                usleep(50*1000);
            }
            std::cout << "\033[34m" << "[Demon_Core_t] 可视化线程结束！" << "\033[0m" << std::endl;
        }
    };
    ThRecv.detach();    
}
