/**
    ENU坐标系(yaw从x到y)----->FRAME_LOCAL_NED(mavros转换)----->NED坐标系    均相对于world

            ↑y(N)             mavros         ↑x(N)
            |               --------->       |
    \       |                                |
  -----> (U)。-------→x(E)               (D) x -------->y(E)
    /
   自定义控制坐标系FRU(相对于base_link or body)

  ↑       ↑x'
  |       |
/ | \     |
  |       。-------->y'


**/
#include "transfer.hpp"

/** 
* @brief   限幅函数 
* @remarks min<data<max
*/
float Limit(float data,float max,float min){
	float Temp = data;
	if(data >= max) Temp = max;
	if(data <= min) Temp = min;
	return Temp;
}


/**
* @brief  构造函数
* @details 
* @par    日志
* @retval none
*/
Transfer::Transfer(std::string config_path):nh("~"){
    readParameters(config_path);
    // 订阅
    vision_odom_sub     = nh.subscribe<nav_msgs::Odometry>          ("/vins_estimator/odometry"     , 1, &Transfer::vision_odom_callback    , this);
    trajectory_pose_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd"         , 1, &Transfer::trajectory_pose_callback, this);
    rc_sub              = nh.subscribe<mavros_msgs::RCIn>           ("/mavros/rc/in"                , 1, &Transfer::rc_callback             , this);
    mavros_imu_sub      = nh.subscribe<sensor_msgs::Imu>            ("/mavros/imu/data"             , 1, &Transfer::mavros_imu_callback     , this);
    mavros_mag_sub      = nh.subscribe<sensor_msgs::MagneticField>  ("/mavros/imu/mag"              , 1, &Transfer::mavros_mag_callback     , this);
    mavros_state_sub    = nh.subscribe<mavros_msgs::State>          ("/mavros/state"                , 1, &Transfer::mavros_state_callback   , this);
    mavros_odom_sub     = nh.subscribe<nav_msgs::Odometry>          ("/mavros/local_position/odom"  , 1, &Transfer::mavros_odom_callback    , this);
    mavros_gps_sub      = nh.subscribe<mavros_msgs::GPSRAW>         ("/mavros/gpsstatus/gps1/raw"   , 1, &Transfer::mavros_gps_callback     , this);
    car_gps_sub         = nh.subscribe<sensor_msgs::NavSatFix>      ("/car/pos/gps"                 , 1, &Transfer::car_gps_callback        , this);
    human_pos_sub       = nh.subscribe<geometry_msgs::PoseStamped>  ("/human/pos/world"             , 1, &Transfer::human_pos_callback      , this);
    track_pos_sub       = nh.subscribe<geometry_msgs::PoseStamped>  ("/human/pose/track"            , 1, &Transfer::track_pos_callback      , this);
    keyboard_pos_sub    = nh.subscribe<geometry_msgs::PoseStamped>  ("/keyboard/pose"               , 1, &Transfer::keyboard_pos_callback   , this);

    // 发布
    setpoint_pub      = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1); // 注意要将mavros中setpoint_raw放进白名单，可以在/mavros/setpoint_raw/target_local中查看响应
    camera_pose_pub   = nh.advertise<geometry_msgs::PoseStamped> ("/stereo/pose", 1);
    drone_odom_pub    = nh.advertise<nav_msgs::Odometry>         ("/mavros/odometry/out", 1);
    drone_pose_pub    = nh.advertise<geometry_msgs::PoseStamped> ("/mavros/vision_pose/pose", 1);
    // drone_vel_pub     = nh.advertise<geometry_msgs::TwistStamped>("/mavros/vision_speed/speed_twist", 1);
    drone_vel_pub     = nh.advertise<geometry_msgs::Vector3Stamped>("/mavros/vision_speed/speed_vector", 1);
    vins_restart_pub  = nh.advertise<std_msgs::Bool>             ("/vins_restart", 1);
    waypoint_pub      = nh.advertise<geometry_msgs::PoseStamped> ("/move_base_simple/goal", 1);
    vis_drone_pub     = nh.advertise<visualization_msgs::Marker> ("/visualization/drone_model", 10, true);
    human_pos_init_pub= nh.advertise<geometry_msgs::PoseStamped> ("/human/pose/init", 1);
    human_pos_gps_pub = nh.advertise<sensor_msgs::NavSatFix>     ("/human/pose/gps", 1);

    // fp_path = fopen((std::string(getenv("HOME"))+"/Escort_Drone_ws/output/transfer.csv").c_str(), "w+");
    // fprintf(fp_path, "时间,目标位置(w).x,目标位置(w).y,目标位置(w).z\n");

    // setPoint_fp = fopen((std::string(getenv("HOME"))+"/Escort_Drone_ws/output/setpoint.csv").c_str(), "w+");
    // fprintf(setPoint_fp, "时间,目标位置(w).x,目标位置(w).y,目标位置(w).z,无人机位置(w).x,无人机位置(w).y,无人机位置(w).z\n");
    
    // gps_fp = fopen((std::string(getenv("HOME"))+"/Escort_Drone_ws/output/drone_gps_pos.csv").c_str(), "w+");
    // human_gps_fp = fopen((std::string(getenv("HOME"))+"/Escort_Drone_ws/output/human_gps_pos.csv").c_str(), "w+");
    // car_gps_fp = fopen((std::string(getenv("HOME"))+"/Escort_Drone_ws/output/car_gps_pos.csv").c_str(), "w+");

    std::thread * setpoint_thread_  = new std::thread(&Transfer::Remote_Handle  , this); // offboard目标的发送线程
    std::thread * tf_thread_        = new std::thread(&Transfer::tf_process     , this); // 坐标变换的除了和发送线程
    COUTG("[INFO] transfer_node launched.\n");
}


void Transfer::readParameters(std::string config_path){
    COUTG("[INFO] 开始读取配置文件: " << config_path);
    FILE *fh = fopen(config_path.c_str(),"r");
    if(fh == NULL){
        COUTR("[ERRO] config_file dosen't exist; wrong config_file path: " << config_path);
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_path.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened()){
        COUTR("[ERRO] failed to open file: " << config_path);
        return;
    }
    fsSettings["ctrFreq"]          >> parm.ctrFreq;
    fsSettings["if_showInfo"]      >> parm.ifShowInfo;
    fsSettings["if_useSelfApp"]    >> parm.useSelfApp;
    fsSettings["pid_xy_p"]         >> parm.pid_xy_p;
    fsSettings["pid_z_p"]          >> parm.pid_z_p;
    fsSettings["pid_yaw_p"]        >> parm.pid_yaw_p;
    fsSettings["max_xy_vel"]       >> parm.max_xy_vel;
    fsSettings["max_z_vel"]        >> parm.max_z_vel;
    
    fsSettings["target_pose_w_sub_topic"]   >> parm.target_pose_w_sub_topic;

    fsSettings["imu_T_body"]       >> parm.T_ib_mat;
    fsSettings["imu_T_cam0"]       >> parm.T_ic_mat;
    fsSettings.release();

    parm.T_ib = Eigen::Isometry3d::Identity();
    parm.T_ic = Eigen::Isometry3d::Identity();
    for(int i=0; i<4; i++){
        for(int j=0; j<4; j++){
            parm.T_ib(i,j) = parm.T_ib_mat.at<double>(i,j);
            parm.T_ic(i,j) = parm.T_ic_mat.at<double>(i,j);
        }
    }

    COUTG("[INFO] 配置文件读取结束！");

    parm.RC_Z_RATIO  = 5.0f*50.0f/parm.ctrFreq;
    parm.RC_XY_RATIO = 10.0f*50.0f/parm.ctrFreq;
    parm.RC_YAW_RATIO= 6.0f*50.0f/parm.ctrFreq;
}

void Transfer::tf_process(void){
    int interval_t = 1000 / 30;    // 50Hz
    vins_ready.lock();

    while(ros::ok()){
        vins_ready.lock();
        static double last_t = 0;
        double tNow = ros::Time::now().toSec();
        if((tNow - last_t) != 0) vision_odom_freq = vision_odom_freq * 0.93 + 0.07 / (tNow - last_t);
        last_t = tNow;

        T_wb = Eigen::Isometry3d::Identity();
        T_wc = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d T_wi = Eigen::Isometry3d::Identity();
        T_wi.rotate(Eigen::Quaterniond(Eigen::Vector4d(vins_odom.pose.pose.orientation.x, vins_odom.pose.pose.orientation.y, vins_odom.pose.pose.orientation.z, vins_odom.pose.pose.orientation.w)).toRotationMatrix());
        T_wi.pretranslate(Eigen::Vector3d (vins_odom.pose.pose.position.x, vins_odom.pose.pose.position.y, vins_odom.pose.pose.position.z));
        T_wb = T_wi * parm.T_ib;
        T_wc = T_wi * parm.T_ic;

        /* 相机位置转换 */
        Eigen::Quaterniond q_temp = Eigen::Quaterniond(T_wc.rotation());
        Eigen::Vector3d    t_temp = T_wc.translation();
        camera_pose.header.frame_id = "world";
        camera_pose.header.stamp = vins_odom.header.stamp;
        camera_pose.pose.position.x = t_temp[0];
        camera_pose.pose.position.y = t_temp[1];
        camera_pose.pose.position.z = t_temp[2]; 
        camera_pose.pose.orientation.x = q_temp.x();
        camera_pose.pose.orientation.y = q_temp.y();
        camera_pose.pose.orientation.z = q_temp.z();
        camera_pose.pose.orientation.w = q_temp.w();

        /* 无人机位置转换 */
        q_temp = Eigen::Quaterniond(T_wb.rotation());
        t_temp = T_wb.translation();    
        drone_odom.header.frame_id = "odom";      // 坐标tf必须这样写
        drone_odom.child_frame_id = "base_link";
        drone_odom.header.stamp = vins_odom.header.stamp;
        drone_odom.pose.pose.position.x = t_temp[0];
        drone_odom.pose.pose.position.y = t_temp[1];
        drone_odom.pose.pose.position.z = t_temp[2];
        drone_odom.pose.pose.orientation.x = q_temp.x();
        drone_odom.pose.pose.orientation.y = q_temp.y();
        drone_odom.pose.pose.orientation.z = q_temp.z();
        drone_odom.pose.pose.orientation.w = q_temp.w();

        geometry_msgs::Vector3 euler;
        tf::Matrix3x3 mat;
        mat = tf::Matrix3x3(tf::Quaternion(drone_odom.pose.pose.orientation.x, drone_odom.pose.pose.orientation.y, drone_odom.pose.pose.orientation.z, drone_odom.pose.pose.orientation.w));
        mat.getRPY(euler.x, euler.y, euler.z);

        float temp_z = euler.z;
        drone_odom.twist.twist.linear.x = vins_odom.twist.twist.linear.x * cosf(temp_z) + vins_odom.twist.twist.linear.y * sinf(temp_z);
        drone_odom.twist.twist.linear.y = -vins_odom.twist.twist.linear.x * sinf(temp_z) + vins_odom.twist.twist.linear.y * cosf(temp_z);
        drone_odom.twist.twist.linear.z = vins_odom.twist.twist.linear.z;
        // drone_odom.twist.twist.linear = vins_odom.twist.twist.linear;
        // drone_odom.twist.twist.linear.x = 1;
        
        geometry_msgs::PoseStamped  drone_pose;
        drone_pose.header.stamp = vins_odom.header.stamp;
        drone_pose.header.frame_id = "odom";      // 坐标tf必须这样写
        drone_pose.pose.position.x = t_temp[0];
        drone_pose.pose.position.y = t_temp[1];
        drone_pose.pose.position.z = t_temp[2];
        drone_pose.pose.orientation.x = q_temp.x();
        drone_pose.pose.orientation.y = q_temp.y();
        drone_pose.pose.orientation.z = q_temp.z();
        drone_pose.pose.orientation.w = q_temp.w();
        // drone_pose_pub.publish(drone_pose);

        // geometry_msgs::TwistStamped drone_vel;
        // drone_vel.header.stamp = vins_odom.header.stamp;
        // drone_vel.twist.linear.x = vins_odom.twist.twist.linear.x * cosf(temp_z) + vins_odom.twist.twist.linear.y * sinf(temp_z);
        // drone_vel.twist.linear.y = -vins_odom.twist.twist.linear.x * sinf(temp_z) + vins_odom.twist.twist.linear.y * cosf(temp_z);
        // drone_vel.twist.linear.z = vins_odom.twist.twist.linear.z;
        geometry_msgs::Vector3Stamped drone_vel;
        drone_vel.header.stamp = vins_odom.header.stamp;
        drone_vel.vector.x = vins_odom.twist.twist.linear.x * cosf(temp_z) + vins_odom.twist.twist.linear.y * sinf(temp_z);
        drone_vel.vector.y = -vins_odom.twist.twist.linear.x * sinf(temp_z) + vins_odom.twist.twist.linear.y * cosf(temp_z);
        drone_vel.vector.z = vins_odom.twist.twist.linear.z;
        drone_vel_pub.publish(drone_vel);

        camera_pose_pub.publish(camera_pose);
        drone_odom_pub.publish(drone_odom);
        vis_drone_model();
        
        // int wait_t = interval_t - (ros::Time::now().toSec() - tNow)*1000;
        // if(wait_t > 0)  std::this_thread::sleep_for(std::chrono::milliseconds(wait_t));  
    }
}

/**
* @brief  遥控器回调
* @details 进行位姿转换并发布
* @par    日志
* @retval none
*/
void Transfer::rc_callback(const mavros_msgs::RCIn::ConstPtr& msg){
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
        // 更新mode
        if(rc.SWA) mode = Safe_Mode;
        else if(rc.SWD){
            if(mavros_state.mode != "OFFBOARD") COUTY("[WARN] Waiting to change to OFFBOARD mode!");
            // else if(rc.SWC == 2)    mode = Turn_Back_Mode;
            else{
                if(rc.SWB == 0)      mode = RC_ctr_Mode;
                else if(rc.SWB == 1) mode = Keyboard_ctr_mode;
                else if(rc.SWB == 2) mode = Traj_ctr_Mode;
            }
        }
        else{
            if(rc.SWC == 0)      mode = Stabilized_Mode;    // 自稳
            else if(rc.SWC == 1) mode = Position_Mode;      // 定点模式
            else if(rc.SWC == 2) mode = Land_Mode;          // 降落模式
        }
    }
    else{
        std::cout << "\033[1;31m" << "[ERRO] Please check your RC!!，通道数：" << msg->channels.size() << "\033[0m" << std::endl;
    }
}

/**
* @brief   键盘数据回调
* @details 
* @par    日志
* @retval none
*/
void Transfer::keyboard_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    COUTB("[INFO] keyboard data recv!");
    bool if_track_ready = false;
    
    if(msg->header.frame_id == "world"){
        track_pos.pose.position = msg->pose.position;
        if_track_ready = true;
    } 
    else if(msg->header.frame_id == "GPS"){
        Eigen::Isometry3d T_wh = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d T_gh = Eigen::Isometry3d::Identity();
        /**** UTM坐标系下的行人位置 ****/
        geographic_msgs::GeoPointStamped gps_msg;
        gps_msg.header = msg->header;
        gps_msg.position.latitude  = msg->pose.position.y;  // 纬度
        gps_msg.position.longitude = msg->pose.position.x;  // 经度
        gps_msg.position.altitude = 1.5f;      
        geodesy::UTMPoint utm;
        geodesy::fromMsg(gps_msg.position, utm);      
        T_gh.pretranslate(Eigen::Vector3d (utm.easting, utm.northing, utm.altitude));
        /**** 世界坐标系下的行人位置 ****/
        T_wh = T_wb * T_gb.inverse() * T_gh;
        Eigen::Vector3d t_temp = T_wh.translation();
        track_pos.pose.position.x = t_temp[0];
        track_pos.pose.position.y = t_temp[1];
        track_pos.pose.position.z = t_temp[2];
        if_track_ready = true;
    }
    else{ 
        Eigen::Vector3d t_temp = pose_c_2_w(Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
        track_pos.pose.position.x = t_temp[0];
        track_pos.pose.position.y = t_temp[1];
        track_pos.pose.position.z = t_temp[2];
        if_track_ready = true;
    }

    if(if_track_ready){
        track_pos.header.frame_id = "world";
        track_pos.pose.orientation.x = msg->pose.orientation.x;
        if((int)msg->pose.orientation.x == 1){
            COUTB("[INFO] 初始化跟踪位置：(" << track_pos.pose.position.x << ", " << track_pos.pose.position.y << ", " << track_pos.pose.position.z << ")");            
        }
        human_pos_init_pub.publish(track_pos);
        target_pos_ready = true;
    }
}

/**
* @brief   GPS数据回调
* @details 无人机GWS84转UTM坐标，并发布；
* @par    日志
* @retval none
*/
void Transfer::mavros_gps_callback(const mavros_msgs::GPSRAW::ConstPtr& msg){      // mavros gps数据回调 
    /**** GWS84转UTM ****/
    // geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
    gps_msg.header = msg->header;
    gps_msg.position.latitude = (float)msg->lat/10000000.0f;
    gps_msg.position.longitude = (float)msg->lon/10000000.0f;
    gps_msg.position.altitude = (float)msg->alt/1000.0f;      
    geodesy::UTMPoint utm;
    geodesy::fromMsg(gps_msg.position, utm);

    /**** 发送无人机UTM坐标（偏置后） ****/
    satellites_num = msg->satellites_visible;
    // 卫星里程计初始化
    if (!gnss_init && satellites_num >= 8)
    {
        gnss_init = true;
        GPS_init_pos[0] = utm.easting;
        GPS_init_pos[1] = utm.northing;
        GPS_init_pos[2] = utm.altitude;
    }
    if(gnss_init){
        GPS_odom.header = msg->header;
        GPS_odom.pose.pose.position.x = utm.easting  - GPS_init_pos[0];
        GPS_odom.pose.pose.position.y = utm.northing - GPS_init_pos[1];
        GPS_odom.pose.pose.position.z = utm.altitude - GPS_init_pos[2];
        tf::Quaternion q = tf::createQuaternionFromYaw(mag_yaw);
        GPS_odom.pose.pose.orientation.x = q.x();
        GPS_odom.pose.pose.orientation.y = q.y();
        GPS_odom.pose.pose.orientation.z = q.z();
        GPS_odom.pose.pose.orientation.w = q.w();
        // fprintf(gps_fp, "%lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf\n", GPS_odom.header.stamp.toSec(), GPS_odom.pose.pose.position.x, GPS_odom.pose.pose.position.y, GPS_odom.pose.pose.position.z,
        //         GPS_odom.pose.pose.orientation.w, GPS_odom.pose.pose.orientation.x, GPS_odom.pose.pose.orientation.y, GPS_odom.pose.pose.orientation.z);  
    }

    if(human_pose_ready){
        human_pose_ready = false;
        T_gb = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d T_wh = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d T_bw = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d T_gh = Eigen::Isometry3d::Identity();

        /**** T_gb获取 ****/
        T_gb.pretranslate(Eigen::Vector3d (utm.easting, utm.northing, drone_odom.pose.pose.position.z));
        T_gb.rotate(Eigen::Quaterniond(Eigen::Vector4d(GPS_odom.pose.pose.orientation.x, GPS_odom.pose.pose.orientation.y, GPS_odom.pose.pose.orientation.z, GPS_odom.pose.pose.orientation.w)).toRotationMatrix());    
        /**** T_wh获取 ****/
        T_wh = Eigen::Isometry3d::Identity();
        T_wh.pretranslate(Eigen::Vector3d (human_pos.pose.position.x, human_pos.pose.position.y, human_pos.pose.position.z));
        /**** T_bw获取 ****/
        T_bw = T_wb.inverse();
        /**** 计算T_gh ****/
        T_gh = T_gb * T_bw * T_wh;
        /**** 计算行人GPS坐标 ****/
        Eigen::Vector3d t_temp = T_gh.translation();
        utm.easting = t_temp[0];
        utm.northing = t_temp[1];
        utm.altitude = t_temp[2];
        if(gnss_init){
            // fprintf(human_gps_fp, "%lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf\n", GPS_odom.header.stamp.toSec(), utm.easting  - GPS_init_pos[0], utm.northing - GPS_init_pos[1], utm.altitude,
            //     1, 0, 0, 0); 
        } 
        // std::cout << "drone_pos_g:" << utm.easting << ", " << utm.northing << ", " << utm.altitude << std::endl;
        // std::cout << "human_pos_g:" << human_utm.easting << ", " << human_utm.northing << ", " << human_utm.altitude << std::endl;
        geographic_msgs::GeoPoint human_gps = geodesy::toMsg(utm);
        /**** 发布行人GPS坐标 ****/
        human_pos_gps.header = msg->header;
        human_pos_gps.latitude = human_gps.latitude;
        human_pos_gps.longitude = human_gps.longitude;
        human_pos_gps.altitude = human_gps.altitude;
        human_pos_gps_pub.publish(human_pos_gps);
        // std::cout << "drone_GPS:" << gps_msg.position.latitude << ", " << gps_msg.position.longitude << ", " << gps_msg.position.altitude << std::endl;
        // std::cout << "human_GPS:" << human_pos_gps.latitude  << ", " << human_pos_gps.longitude << ", " << human_pos_gps.altitude << std::endl;
        // std::cout << std::endl;
    }
}

Eigen::Vector3d Transfer::pose_c_2_w(Eigen::Vector3d pos_c){
    Eigen::Isometry3d T_bh = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_wh = Eigen::Isometry3d::Identity();
    T_bh.pretranslate(pos_c);
    T_wh = T_wb * T_bh;
    return T_wh.translation();
}

/**
* @brief  模式运行定时器
* @details 进行模式切换和控制点发布
* @par    日志
* @retval none
*/
void Transfer::setpoint_pub_fun(bool if_record){
    if(!parm.useSelfApp){
        setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 但是需要发送ENU坐标系数据，mavros会自动转换的
        setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        // fprintf(setPoint_fp, "%lf\t, [%s], %.2lf\t, %.2lf\t, %.2lf\t, %.2lf\t, %.2lf\t, %.2lf\n", setpoint.header.stamp.toSec(), mode_to_string(mode), 
        //                         setpoint.position.x, setpoint.position.y, setpoint.position.z,
        //                         drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z);
        setpoint_pub.publish(setpoint);
    }
    else{
        mavros_msgs::PositionTarget _setpoint;   
        _setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        _setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | 
                              mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_YAW;

        setpoint.velocity.x = _setpoint.velocity.x = Limit(parm.pid_xy_p * (setpoint.position.x - drone_odom.pose.pose.position.x), parm.max_xy_vel, -parm.max_xy_vel);
        setpoint.velocity.y = _setpoint.velocity.y = Limit(parm.pid_xy_p * (setpoint.position.y - drone_odom.pose.pose.position.y), parm.max_xy_vel, -parm.max_xy_vel);
        setpoint.velocity.z = _setpoint.velocity.z = Limit(parm.pid_z_p  * (setpoint.position.z - drone_odom.pose.pose.position.z), parm.max_z_vel,  -parm.max_z_vel);

        float tempErr0 = setpoint.yaw - (float)tf::getYaw(drone_odom.pose.pose.orientation);
        float tempErr1 = tempErr0 - 2 * M_PI * SIGN(tempErr0);
        setpoint.yaw_rate = _setpoint.yaw_rate = parm.pid_yaw_p * abs(tempErr0)>abs(tempErr1)?tempErr1:tempErr0;    

        _setpoint.acceleration_or_force.x = setpoint.acceleration_or_force.x;
        _setpoint.acceleration_or_force.y = setpoint.acceleration_or_force.y;
        _setpoint.acceleration_or_force.z = setpoint.acceleration_or_force.z;

        setpoint_pub.publish(_setpoint);
    }
}

/**
* @brief  数据打印
* @detail 
* @par    日志
* @retval none
*/
void Transfer::showInfo(void){
    geometry_msgs::Vector3 euler;
    tf::Matrix3x3 mat;
    //固定的浮点显示
    std::cout.setf(std::ios::fixed);
    std::cout << std::setprecision(2); //设显示小数精度为n位
    //左对齐
    std::cout.setf(std::ios::left);
    // 强制显示小数点
    std::cout.setf(std::ios::showpoint);
    // 强制显示符号
    std::cout.setf(std::ios::showpos);
    COUTG("[INFO] vins_freq: ：" << vision_odom_freq << " [Hz] ");    
    COUTG("[INFO] 当前模式为：" << mode_to_string(mode));
    COUTG("[INFO] 当前GPS卫星数为：" << satellites_num);
    // std::cout << "vins_freq: " << vision_odom_freq << " [Hz] " << std::endl; 
    if(gnss_init){
        
    }
    else COUTG("[INFO] GPS未初始化!");

    // std::cout <<">>>>>>>>>>>>>>>>>>>>>>>> Human Info [ENU Frame] <<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
    // std::cout << std::setprecision(7); //设显示小数精度为n位
    // std::cout << "human_GPS  [lon lat]: " << human_pos_gps.longitude << " [°] " << human_pos_gps.latitude << " [°] " << std::endl;
    // std::cout << std::setprecision(2); //设显示小数精度为n位
    // std::cout << "human_pose_w [x y z]: " << human_pos.pose.position.x << " [ m ] " << human_pos.pose.position.y << " [ m ] " << human_pos.pose.position.z << " [ m ] " << std::endl;

    std::cout <<">>>>>>>>>>>>>>>>>>>>>>>> Camera_pose Info [ENU Frame] <<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
    std::cout << "vins_freq: " << vision_odom_freq << " [Hz] " << std::endl; 
    std::cout << "camera_pose [x y z]: " << camera_pose.pose.position.x << " [ m ] " << camera_pose.pose.position.y << " [ m ] " << camera_pose.pose.position.z << " [ m ] " << std::endl;
    mat = tf::Matrix3x3(tf::Quaternion(camera_pose.pose.orientation.x, camera_pose.pose.orientation.y, camera_pose.pose.orientation.z, camera_pose.pose.orientation.w));
    mat.getRPY(euler.x, euler.y, euler.z);
    std::cout << "camera_euler[R P Y]: " << euler.x/M_PI*180.0 << " [deg] " << euler.y/M_PI*180.0 << " [deg] " << euler.z/M_PI*180.0 << " [deg] " << std::endl;

    std::cout <<">>>>>>>>>>>>>>>>>>>>>>>> Drone Info [ENU Frame] <<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
    std::cout << std::setprecision(7); //设显示小数精度为n位
    std::cout << "Drone_GPS [lon lat alt]: " << gps_msg.position.longitude << " [°] " << gps_msg.position.latitude << " [°] " << std::setprecision(2) << gps_msg.position.altitude << " [ m ] " << std::endl;
    // std::cout << "Drone_pose_g [x y z]: " << GPS_odom.pose.pose.position.x << " [ m ] " << GPS_odom.pose.pose.position.y << " [ m ] " << GPS_odom.pose.pose.position.z << " [ m ] " << std::endl;
    std::cout << "Drone_pose_w [x y z]: " << drone_odom.pose.pose.position.x << " [ m ] " << drone_odom.pose.pose.position.y << " [ m ] " << drone_odom.pose.pose.position.z << " [ m ] " << std::endl;
    std::cout << "Drone_vel    [x y z]: " << drone_odom.twist.twist.linear.x << " [m/s] " << drone_odom.twist.twist.linear.y << " [m/s] " << drone_odom.twist.twist.linear.z << " [m/s] " << std::endl;
    mat = tf::Matrix3x3(tf::Quaternion(drone_odom.pose.pose.orientation.x, drone_odom.pose.pose.orientation.y, drone_odom.pose.pose.orientation.z, drone_odom.pose.pose.orientation.w));
    mat.getRPY(euler.x, euler.y, euler.z);
    std::cout << "Drone_euler  [R P Y]: " << euler.x/M_PI*180.0 << " [deg] " << euler.y/M_PI*180.0 << " [deg] " << euler.z/M_PI*180.0 << " [deg] " << std::endl;

    std::cout <<">>>>>>>>>>>>>>>>>>>>>>>> mavros_odom Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
    // std::cout << "mavros_freq: " << mavros_odom_freq << " [Hz] " << std::endl; 
    std::cout << "mavros_pose [x y z]: " << mavros_odom.pose.pose.position.x << " [ m ] " << mavros_odom.pose.pose.position.y << " [ m ] " << mavros_odom.pose.pose.position.z << " [ m ] " << std::endl;
    std::cout << "mavros_vel  [x y z]: " << mavros_odom.twist.twist.linear.x << " [m/s] " << mavros_odom.twist.twist.linear.y << " [m/s] " << mavros_odom.twist.twist.linear.z << " [m/s] " << std::endl;
    mat = tf::Matrix3x3(tf::Quaternion(mavros_odom.pose.pose.orientation.x, mavros_odom.pose.pose.orientation.y, mavros_odom.pose.pose.orientation.z, mavros_odom.pose.pose.orientation.w));
    mat.getRPY(euler.x, euler.y, euler.z);
    std::cout << "mavros_euler[R P Y]: " << euler.x/M_PI*180.0 << " [deg] " << euler.y/M_PI*180.0 << " [deg] " << euler.z/M_PI*180.0 << " [deg] " << std::endl;

    // std::cout <<">>>>>>>>>>>>>>>>>>>>>>>> mavros_imu Info <<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
    // mat = tf::Matrix3x3(tf::Quaternion(mavros_imu.orientation.x, mavros_imu.orientation.y, mavros_imu.orientation.z, mavros_imu.orientation.w));
    // mat.getRPY(euler.x, euler.y, euler.z);
    // std::cout << "mavros_imu[R P Y]: " << euler.x/M_PI*180.0 << " [deg] " << euler.y/M_PI*180.0 << " [deg] " << mag_yaw/M_PI*180.0 << " [deg] " << std::endl;

    // std::cout <<">>>>>>>>>>>>>>>>>>>>>>>> setpoint Info [ENU Frame] <<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
    // std::cout << "setpoint_pose [x y z yaw]: " << setpoint.position.x << " [ m ] " << setpoint.position.y << " [ m ] " << setpoint.position.z << " [ m ] " << setpoint.yaw/M_PI*180.0 << " [deg] " << std::endl;
    // std::cout << "setpoint_vel  [x y z yaw]: " << setpoint.velocity.x << " [m/s] " << setpoint.velocity.y << " [m/s] " << setpoint.velocity.z << " [m/s] " << setpoint.yaw_rate/M_PI*180.0 << " [deg/s] " << std::endl;
    // std::cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> END <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
    std::cout << std::endl;    
}

/**
* @brief  模式控制器
* @detail 周期执行，进行模式调度
* @retval  0 OK
* @log    返回值目前无效
* @par 日志  2018年12月13日16:23:40  修改mode和last_mode为static型，解决不停进入if的问题
*/
void Transfer::Remote_Handle(void){
    COUTG("[INFO] offboard 目标点发送线程启动！当前模式为：" << mode_to_string(mode));
    target_pos_ready = false;
    while(ros::ok()){
        // 判断mode是否更新
        if(mode != last_mode){
            // 退出之前的模式
            COUTB("[INFO] exit the mode : " << mode_to_string(last_mode));
            // 启用当前模式
            COUTB("[INFO] change to mode: " << mode_to_string(mode));
            if(mode == Traj_ctr_Mode){
                Eigen::Vector3d t_temp = pose_c_2_w(Eigen::Vector3d(8.0, 0.0, 0.8));
                track_pos.pose.position.x = t_temp[0];
                track_pos.pose.position.y = t_temp[1];
                track_pos.pose.position.z = t_temp[2];
                track_pos.pose.orientation.x = 1;
                track_pos.header.frame_id = "world";
                target_pos_ready = true;
                human_pos_init_pub.publish(track_pos);
                COUTB("[INFO] 初始化跟踪位置：(" << track_pos.pose.position.x << ", " << track_pos.pose.position.y << ", " << track_pos.pose.position.z << ")");            
            }
            if(mode == Turn_Back_Mode){
                track_pos.pose.position.x = -2;
                track_pos.pose.position.y = -2;
                track_pos.pose.position.z = drone_odom.pose.pose.position.z + 0.8;
                track_pos.pose.orientation.x = 0;
                track_pos.header.frame_id = "world";
                human_pos_init_pub.publish(track_pos);
                target_pos_ready = true;
            }
            last_mode = mode;
        }

        if(mode == RC_ctr_Mode){
            static bool hover_flag = false;      // 主要为了解决位置控制存在延时的问题
            if(rc.Pitch != 0 || rc.Roll != 0 || rc.Yaw != 0 || rc.Throttle != 0){
                if(rc.Yaw != 0){
                    static float last_targetYaw = 0;
                    last_targetYaw = setpoint.yaw;
                    setpoint.yaw -= rc.Yaw/200000.0*parm.RC_YAW_RATIO;

                    if(abs(setpoint.yaw) > M_PI)  setpoint.yaw -= 2*M_PI*SIGN(setpoint.yaw - last_targetYaw);
                }
                if(rc.Pitch != 0 || rc.Roll != 0){
                    setpoint.position.x += (rc.Pitch/200000.0*cosf(setpoint.yaw)+rc.Roll/200000.0*sinf(setpoint.yaw))*parm.RC_XY_RATIO;
                    setpoint.position.y += (rc.Pitch/200000.0*sinf(setpoint.yaw)-rc.Roll/200000.0*cosf(setpoint.yaw))*parm.RC_XY_RATIO;
                    setpoint.position.x = Limit(setpoint.position.x, drone_odom.pose.pose.position.x+0.5, drone_odom.pose.pose.position.x-0.5);
                    setpoint.position.y = Limit(setpoint.position.y, drone_odom.pose.pose.position.y+0.5, drone_odom.pose.pose.position.y-0.5);
                }
                if(rc.Throttle != 0){
                    setpoint.position.z += rc.Throttle/400000.0*parm.RC_Z_RATIO;
                    setpoint.position.z = Limit(setpoint.position.z, drone_odom.pose.pose.position.z+0.5, drone_odom.pose.pose.position.z-0.5);
                }
                setpoint.velocity.x = 0;
                setpoint.velocity.y = 0;
                setpoint.velocity.z = 0;
                setpoint.acceleration_or_force.x = 0;
                setpoint.acceleration_or_force.y = 0;
                setpoint.acceleration_or_force.z = 0;
                setpoint.yaw_rate = 0;
                hover_flag = true;
            }
            else if(hover_flag){          // 当松开遥控器，立刻悬停
                setpoint.position.x = Limit(setpoint.position.x, drone_odom.pose.pose.position.x+0.4, drone_odom.pose.pose.position.x-0.4);
                setpoint.position.y = Limit(setpoint.position.y, drone_odom.pose.pose.position.y+0.4, drone_odom.pose.pose.position.y-0.4);
                setpoint.position.z = Limit(setpoint.position.z, drone_odom.pose.pose.position.z+0.4, drone_odom.pose.pose.position.z-0.4);
                hover_flag = false;
            }
            setpoint_pub_fun(false);
            if(parm.ifShowInfo) showInfo();
            std::chrono::milliseconds dura(1000/parm.ctrFreq);
            std::this_thread::sleep_for(dura);              
        }
        else if(mode == Keyboard_ctr_mode || mode == Traj_ctr_Mode || mode == Turn_Back_Mode){
            static double pub_t = ros::Time::now().toSec();
            if(mode == Traj_ctr_Mode && parm.ifShowInfo) showInfo();
            if(target_pos_ready){
                COUTB("[INFO] 发送跟踪位置：(" << track_pos.pose.position.x << ", " << track_pos.pose.position.y << ", " << track_pos.pose.position.z << ")"); 
                // fprintf(fp_path, "%lf\t, %.2lf\t, %.2lf\t, %.2lf, %lf\t, %.2lf\t, %.2lf\t, %.2lf\n", track_pos.header.stamp.toSec(), track_pos.pose.position.x, track_pos.pose.position.y, track_pos.pose.position.z
                //                 , drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z);
                waypoint_pub.publish(track_pos);  // 向路径规划发送目标点
                target_pos_ready = false;            
            }
            if(trajectory_pose_ready){
                setpoint.position = trajectory_pose.position;
                setpoint.velocity = trajectory_pose.velocity;
                setpoint.acceleration_or_force = trajectory_pose.acceleration;
                setpoint.yaw = trajectory_pose.yaw;
                setpoint.yaw_rate = trajectory_pose.yaw_dot;
                trajectory_pose_ready = false;
                pub_t = ros::Time::now().toSec();
                setpoint_pub_fun();
            }
            if((ros::Time::now().toSec() - pub_t) > 1.0/parm.ctrFreq){
                setpoint_pub_fun();
                pub_t = ros::Time::now().toSec();
            }
            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);         
        }
        else{  // 遥控器在控制或者未解锁
            if(!mavros_state.armed && rc.Throttle > 50)
                COUTY("[WARN] Please arming first!!!!");
            setpoint.position = drone_odom.pose.pose.position;
            setpoint.yaw = (float)tf::getYaw(drone_odom.pose.pose.orientation);
            setpoint.velocity.x = 0;
            setpoint.velocity.y = 0;
            setpoint.velocity.z = 0;
            setpoint.acceleration_or_force.x = 0;
            setpoint.acceleration_or_force.y = 0;
            setpoint.acceleration_or_force.z = 0;
            setpoint.yaw_rate = 0;
            setpoint_pub_fun(false);
            if(parm.ifShowInfo) showInfo();
            std::chrono::milliseconds dura(1000/parm.ctrFreq);
            std::this_thread::sleep_for(dura);
        }
    }
}

std::string Transfer::mode_to_string(uint16_t _mode){
    std::string res;
    if(_mode == Safe_Mode) res = "Safe_Mode";
    else if(_mode == Stabilized_Mode)   res = "Stabilized_Mode";
    else if(_mode == Position_Mode)     res = "Position_Mode";
    else if(_mode == Land_Mode)         res = "Land_Mode";
    else if(_mode == RC_ctr_Mode)       res = "RC_ctr_Mode";
    else if(_mode == Keyboard_ctr_mode) res = "Keyboard_ctr_mode";
    else if(_mode == Traj_ctr_Mode)     res = "Traj_ctr_Mode";
    else if(_mode == Turn_Back_Mode)    res = "Turn_Back_Mode";
    else res = "ERRO";
    return res;
}

/**
* @brief   可视化无人机模型
* @details 
* @par    日志
* @retval none
*/
void Transfer::vis_drone_model(){
    /* 发布可视化飞机模型 */
    visualization_msgs::Marker planeMarker;
    planeMarker.id = 0;
    planeMarker.header.stamp = drone_odom.header.stamp;
    planeMarker.header.frame_id = "world";
    planeMarker.action = visualization_msgs::Marker::ADD;
    planeMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
    planeMarker.ns = "plane_mesh";
    planeMarker.mesh_use_embedded_materials = true;
    planeMarker.color.r = 0.0;
    planeMarker.color.g = 0.0;
    planeMarker.color.b = 0.0;
    planeMarker.color.a = 0.0;
    planeMarker.scale.x = 0.0005;
    planeMarker.scale.y = 0.0005;
    planeMarker.scale.z = 0.0005;

    tf::Quaternion q1(0, 0, 0.707, 0.707);
    tf::Quaternion q2(drone_odom.pose.pose.orientation.x, drone_odom.pose.pose.orientation.y,
                      drone_odom.pose.pose.orientation.z, drone_odom.pose.pose.orientation.w);
    tf::Quaternion new_q = q1 * q2;

    geometry_msgs::Vector3 euler;
    tf::Matrix3x3 plane_mat(tf::Quaternion(new_q.x(), new_q.y(), new_q.z(), new_q.w()));
    plane_mat.getRPY(euler.x, euler.y, euler.z);  // 获得欧拉角
    geometry_msgs::Quaternion transform_q = tf::createQuaternionMsgFromRollPitchYaw(-euler.x-M_PI/2 , euler.y+M_PI, euler.z+M_PI); //  + M_PI/2    

    planeMarker.pose.orientation.w = transform_q.w;
    planeMarker.pose.orientation.x = transform_q.x;
    planeMarker.pose.orientation.y = transform_q.y;
    planeMarker.pose.orientation.z = transform_q.z;
    planeMarker.pose.position.x = drone_odom.pose.pose.position.x;
    planeMarker.pose.position.y = drone_odom.pose.pose.position.y;
    planeMarker.pose.position.z = drone_odom.pose.pose.position.z;
    planeMarker.mesh_resource = std::string("package://commu_bridge/model/Plane.fbx");   
    vis_drone_pub.publish(planeMarker);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "transfer_node");
    ros::MultiThreadedSpinner spinner(0);
    if(argc != 2)
    {
        COUTR("[ERRO] Please intput config file!");
        return 1;
    }
    std::string config_file_path = argv[1];
    COUTG("[INFO] config_file:" << argv[1]);
    COUTG("[INFO] transfer_node launch!!");

    Transfer transfer(config_file_path);
    spinner.spin();
    ROS_INFO("[INFO] transfer_node closed.\n");
    return 0;
}

