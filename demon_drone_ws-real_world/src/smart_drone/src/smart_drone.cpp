#include "smart_drone.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/tracking/tracker.hpp>

Smart_Drone_t::Smart_Drone_t(void):nh("~")
{
    image_transport::ImageTransport it(nh);  //  类似ROS句柄
    std::string modelPath;
    nh.param<std::string>("modelPath", modelPath, "");
    if(modelPath == "")
    {
        std::cout << "\033[31m " << "[Smart_Drone_t] please input modelPath in smart_drone.launch" << "\033[0m" << std::endl;
    }
    // 订阅
    rc_sub              = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in"    , 1, &Smart_Drone_t::rc_callback, this);
    mavros_imu_sub      = nh.subscribe<sensor_msgs::Imu> ("/mavros/imu/data" , 1, &Smart_Drone_t::mavros_imu_callback     , this);
    left_rect_img_sub   = it.subscribe("/stereo/left/image_rect", 1, &Smart_Drone_t::lrect_img_callback, this);
    // 发布
    setpoint_pub        = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1); // 注意要将mavros中setpoint_raw放进白名单，可以在/mavros/setpoint_raw/target_local中查看响应

    lRect_img_mutex.lock();

    if(people_detect.init(modelPath))
    {
        isReady = true;
        Object_Detect_Thread();
        Drone_Control_Thread();
        std::cout << "\033[33m" << "*************** [Smart_Drone_t] people_detect初始化完成 ***************" << "\033[0m" << std::endl << std::endl; 
    }
}

void Smart_Drone_t::Object_Detect_Thread(void){
    std::thread ThRecv	= std::thread{
        [&](){
            std::cout << "\033[32m" << "[Smart_Drone_t] 目标跟踪线程开始......" << "\033[0m" << std::endl;
            bool isLock = false;        // 是否锁定
            cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create();
            bool ifTargetDisappeared = false;
            int trackID = -1;
            while (ros::ok() && isReady){
                lRect_img_mutex.lock();
                double tNow = ptr->header.stamp.toSec();
                cv::Mat inputFrame = ptr->image.clone();
                if(image_width == 0)    image_width  = inputFrame.cols;
                if(image_height == 0)   image_height = inputFrame.rows;

                // 目标识别
                people_detect.process(inputFrame, false);    // true: 显示图像，false：不显示图像

                if(mode < TRACK)    isLock = false;
                if(mode < TRACK || !isLock) // isLock变量防止切换到锁定模式，但是视野中无目标
                {
                    if(people_detect.people_boxs.size() > 0)    // 锁定离中心最近的物体
                    {
                        target_info.t = tNow;
                        getNearestBox(people_detect.people_boxs, inputFrame.cols / 2, inputFrame.rows / 2, &target_info.chosen_box);
                        cv::Rect2d rect = cv::Rect2d(target_info.chosen_box.x1, target_info.chosen_box.y1, target_info.chosen_box.x2-target_info.chosen_box.x1, target_info.chosen_box.y2-target_info.chosen_box.y1);
                        trackID = target_info.chosen_box.trackID;
                        // 初始化opencv跟踪
                        tracker->init(inputFrame, rect);
                        isLock = true;
                    }
                    // 没有物体不初始化
                }
                else
                {
                    // 预测
                    cv::Rect2d rect_prdict;
                    tracker->update(inputFrame, rect_prdict);

                    // 找出最相似的框
                    bool if_match = false;
                    double min_distance = 1e9;
                    for (int i = 0; i < people_detect.people_boxs.size(); i++)
                    {
                        int delta_x = (people_detect.people_boxs[i].x1 + people_detect.people_boxs[i].x2) / 2 - (target_info.chosen_box.x1 + target_info.chosen_box.x2)/2;
                        int delta_y = (people_detect.people_boxs[i].y1 + people_detect.people_boxs[i].y2) / 2 - (target_info.chosen_box.y1 + target_info.chosen_box.y2)/2;
                        double distance = sqrt(delta_x * delta_x + delta_y * delta_y);

                        float area = (people_detect.people_boxs[i].x2 - people_detect.people_boxs[i].x1) * (people_detect.people_boxs[i].y2 - people_detect.people_boxs[i].y1);
                        float last_area = (target_info.chosen_box.x2 - target_info.chosen_box.x1) * (target_info.chosen_box.y2 - target_info.chosen_box.y1);
                        if(distance < 100 && abs(area-last_area)/last_area < 0.5)    // 30个像素误差，面积差不超过50%
                        {
                            min_distance = distance;
                            target_info.chosen_box = people_detect.people_boxs[i];
                            if_match = true;
                        }

                        if(people_detect.people_boxs[i].trackID == trackID)
                        {
                            target_info.chosen_box = people_detect.people_boxs[i];
                            if_match = true;
                            break;                   
                        }            
                    }

                    target_info.t = tNow;
                    if(if_match)    // 识别到了目标
                    {
                        cv::Rect2d rect(target_info.chosen_box.x1, target_info.chosen_box.y1, target_info.chosen_box.x2-target_info.chosen_box.x1, target_info.chosen_box.y2-target_info.chosen_box.y1); 
                        cv::rectangle(inputFrame, rect, cv::Scalar(0, 0, 255, 255), 2.0);
                        ifTargetDisappeared = false;
                    }
                    else            // 画面中没有物体
                    {
                        if(!ifTargetDisappeared)
                        {
                            ifTargetDisappeared = true;
                            tracker = cv::TrackerCSRT::create();
                            cv::Rect2d rect = cv::Rect2d(target_info.chosen_box.x1, target_info.chosen_box.y1, target_info.chosen_box.x2-target_info.chosen_box.x1, target_info.chosen_box.y2-target_info.chosen_box.y1); 
                            tracker->init(inputFrame, rect);                    
                        }
                        cv::rectangle(inputFrame, rect_prdict, cv::Scalar(255, 255, 255), 2.0); 
                        target_info.chosen_box.x1 = rect_prdict.x;
                        target_info.chosen_box.y1 = rect_prdict.y;
                        target_info.chosen_box.x2 = rect_prdict.x + rect_prdict.width;
                        target_info.chosen_box.y2 = rect_prdict.y + rect_prdict.height;
                    }
                }

                // 绘制线
                cv::line(inputFrame, cv::Point(inputFrame.cols / 2, inputFrame.rows / 2), 
                            cv::Point((target_info.chosen_box.x1+target_info.chosen_box.x2)/2, (target_info.chosen_box.y1+target_info.chosen_box.y2)/2),
                            cv::Scalar(0, 255, 255),
                            2
                        );

                // 绘制圆心
                cv::circle(inputFrame, cv::Point(inputFrame.cols / 2, inputFrame.rows / 2), 30, cv::Scalar(0, 255, 255), 2);

                // 为所有物体画框
                for (int i = 0; i < people_detect.people_boxs.size(); i++)
                {
                    if(people_detect.people_boxs[i].trackID != trackID)
                        rectangle(inputFrame, 
                                cv::Point(people_detect.people_boxs[i].x1, people_detect.people_boxs[i].y1), 
                                cv::Point(people_detect.people_boxs[i].x2, people_detect.people_boxs[i].y2), 
                                cv::Scalar(0, 255, 0, 255), 
                                2.0);
                }
                cv::imshow("识别图像", inputFrame);
                if(!videoWriter.isOpened())
                {
                    videoWriter.open("/home/ubuntu/demon_drone_ws/output/drone.avi"  // 地址
                                            , cv::VideoWriter::fourcc('M', 'J', 'P', 'G')       // 格式
                                            , 10                                                // 帧率
                                            , inputFrame.size());                                      // 尺寸
                }
                
                videoWriter.write(inputFrame);
                int key = cv::waitKey(1);
                if (27 == key || 'q' == key || 'Q' == key) exit(1);               
            }
            std::cout << "\033[34m" << "[Smart_Drone_t] 结束目标跟踪线程！" << "\033[0m" << std::endl;
        }
    };
    ThRecv.detach();
}

void Smart_Drone_t::Drone_Control_Thread(void)
{
    std::thread ThRecv	= std::thread{
        [&](){
            std::cout << "\033[32m" << "[Smart_Drone_t] 飞行控制线程开始......" << "\033[0m" << std::endl;
            int interval_t = 1000 / 50;    // 50Hz
            int state = 0;
            int last_mode = -1;
            float target_area;
            double target_yaw_rate, target_speed_x, target_speed_y, target_speed_z;
            while (ros::ok() && isReady)
            {
                double tNow = ros::Time::now().toSec();
                // 模式更新
                if(rc.SWD)
                {
                    if(rc.SWB == 0) mode = TRACK;
                    else            mode = ATTACK;
                }
                else                mode = MANUAL;

                if(last_mode != mode)
                {
                    last_mode = mode;
                    state = 0;
                    if(mode == TRACK)   target_area = (target_info.chosen_box.x2 - target_info.chosen_box.x1) * (target_info.chosen_box.y2 - target_info.chosen_box.y1);
                    else                target_area = 307200;
                }

                // 使用VX VY VZ YAW_RATE进行控制
                mavros_msgs::PositionTarget setpoint;
                setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;   // need trans body vel to ENU
                setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | 
                                     mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_YAW |
                                     mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ;
                // yaw对准  目标角速度 = P系数 * (图像中心x坐标 - 目标中心x坐标) 
                double target_yaw_rate_temp = 0.001 * (image_width / 2 - (target_info.chosen_box.x1 + target_info.chosen_box.x2) / 2);
                // target_yaw_rate_temp = limit(target_yaw_rate, -M_PI/2, M_PI/2);
                target_yaw_rate = target_yaw_rate * 0.9 + target_yaw_rate_temp * 0.1;
                // std::cout << "target_yaw_rate: " << target_yaw_rate << " and " << state << std::endl;

                // 距离保持(x) 目标前进速度 = P系数 * (目标框面积 - 实际框面积)
                double target_speed_x_temp = 0.0001 * (target_area - (target_info.chosen_box.x2 - target_info.chosen_box.x1) * (target_info.chosen_box.y2 - target_info.chosen_box.y1));
                target_speed_x = target_speed_x * 0.9 + target_speed_x_temp * 0.1;
                
                // 距离保持(y) 目标左右速度 = P系数 * (图像中心x坐标 - 目标中心x坐标)，看似和yaw冲突了，所以二选一，要么调方向要么跟着移动
                double target_speed_y_temp = 0.01 * (image_width / 2 - (target_info.chosen_box.x1 + target_info.chosen_box.x2) / 2);
                target_speed_y = target_speed_y * 0.9 - target_speed_y_temp * 0.1;
                
                // 高度调整 目标上升速度 = P系数 * (图像中心y坐标 - 目标中心y坐标)
                double target_speed_z_temp = 0.01 * (image_height / 2 - (target_info.chosen_box.y1 + target_info.chosen_box.y2) / 2);
                target_speed_z = target_speed_z * 0.9 + target_speed_z_temp * 0.1;

                if(mode == TRACK)          // 跟踪
                {
                    if(state == 0)  // 先yaw对准
                    {
                        setpoint.velocity.x = 0;    // left+
                        setpoint.velocity.y = 0;    // 
                        setpoint.yaw_rate = target_yaw_rate;
                        if(abs(image_width / 2 - (target_info.chosen_box.x1 + target_info.chosen_box.x2) / 2) < 30)
                            state = 1;
                    }
                    else
                    {
                        setpoint.velocity.x = target_speed_x;
                        setpoint.velocity.y = target_speed_y;                        
                        setpoint.yaw_rate = 0;
                    }
                    std::cout << "track mode[x,y]: " << state << ", " << target_speed_x << "," << target_speed_y << "," << setpoint.yaw_rate << std::endl; 
                }
                else if(mode == ATTACK)    // 攻击
                {
                    setpoint.yaw_rate = target_yaw_rate;
                    setpoint.velocity.x = target_speed_x;
                    setpoint.velocity.y = 0;                    
                }

                setpoint.velocity.z = target_speed_z;
                
                // calculate drone yaw
                tf::Matrix3x3 mat = tf::Matrix3x3(tf::Quaternion(mavros_imu.orientation.x, mavros_imu.orientation.y, mavros_imu.orientation.z, mavros_imu.orientation.w));
                geometry_msgs::Vector3 euler;
                mat.getRPY(euler.x, euler.y, euler.z);

                // trans body vel to ENU
                float temp_x = setpoint.velocity.x * cosf(euler.z) + setpoint.velocity.y * sinf(euler.z);
                float temp_y = setpoint.velocity.x * sinf(euler.z) - setpoint.velocity.y * cosf(euler.z);
                setpoint.velocity.x = temp_x;
                setpoint.velocity.y = temp_y;
                setpoint_pub.publish(setpoint);
                int wait_t = interval_t - (ros::Time::now().toSec() - tNow)*1000;
                if(wait_t > 0)  std::this_thread::sleep_for(std::chrono::milliseconds(wait_t));                  
            }
            std::cout << "\033[34m" << "[Smart_Drone_t] 结束飞行控制线程！" << "\033[0m" << std::endl;
        }
    };
    ThRecv.detach();            
}


/**
 * @brief 获取离坐标(x,y)最近的框
 * @note 需要注意输入空列表的情况
*/
double Smart_Drone_t::getNearestBox(std::vector<DetectBox> &boxes, int x, int y, DetectBox* nearestBox)
{
    double min_distance = 1e9;
    for (int i = 0; i < boxes.size(); i++)
    {        
        int delta_x = (boxes[i].x1 + boxes[i].x2) / 2 - x;
        int delta_y = (boxes[i].y1 + boxes[i].y2) / 2 - y;
        double distance = sqrt(delta_x * delta_x + delta_y * delta_y);
        if(distance < min_distance)
        {
            min_distance = distance;
            *nearestBox = people_detect.people_boxs[i];
        }
    }
    return min_distance;
}

/**
* @brief  遥控器回调
* @details 进行位姿转换并发布
* @par    日志
* @retval none
*/
void Smart_Drone_t::rc_callback(const mavros_msgs::RCIn::ConstPtr& msg){
    if(msg->channels.size() >= 8){
        // 获取遥控器值
        rc.SWA = msg->channels[4] < 1500 ? 0 : 1;
        if(msg->channels[5] < 1200) rc.SWB = 0;
        else if(msg->channels[5] < 1700) rc.SWB = 1;
        else rc.SWB = 2;
        if(msg->channels[6] < 1200) rc.SWC = 0;
        else if(msg->channels[6] < 1700) rc.SWC = 1;
        else rc.SWC = 2;
        rc.SWD = msg->channels[7] < 1500 ? 0 : 1;

        rc.Throttle = msg->channels[2] - 1500;
        if(abs(rc.Throttle) < 50)   rc.Throttle = 0;
        rc.Yaw = msg->channels[3] - 1500;
        if(abs(rc.Yaw) < 50)   rc.Yaw = 0;
        rc.Roll = msg->channels[0] - 1500;
        if(abs(rc.Roll) < 50)   rc.Roll = 0;
        rc.Pitch = msg->channels[1] - 1500;
        if(abs(rc.Pitch) < 50)   rc.Pitch = 0;
    }
    else{
        std::cout << "\033[1;31m" << "[ERRO] Please check your RC!!，通道数：" << msg->channels.size() << "\033[0m" << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smart_drone_node");   // ros初始化，定义节点名为
    ros::MultiThreadedSpinner spinner(0);

    Smart_Drone_t smart_drone;

    ROS_INFO("[INFO] smart_drone_node launch.\n");  
    spinner.spin();
    ROS_INFO("[INFO] smart_drone_node closed.\n");    
}
