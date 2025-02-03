#include "offb_node.h"

Offb_control::Offb_control():nh("~"){
    readParameters();
    param.config_from_ros_handle(nh);

    state_sub =             nh.subscribe<mavros_msgs::State>                ("/mavros/state",                   1,  &Offb_control::state_cb,this);
    gps_info =              nh.subscribe<mavros_msgs::GPSRAW>               ("/mavros/gpsstatus/gps1/raw",      1,  &Offb_control::gps_info_cb,this);
    local_pose_info =       nh.subscribe<geometry_msgs::PoseStamped>        ("/mavros/local_position/pose",     1,  &Offb_control::local_info_cb,this);
    vins_sub =              nh.subscribe<nav_msgs::Odometry>                ("/vins_estimator/odometry",        1,  &Offb_control::vins_cb,this);
    rel_altitude =          nh.subscribe<mavros_msgs::Altitude>             ("/mavros/altitude",                1,  &Offb_control::alt_cb,this);
    px4_quaternion_sub =    nh.subscribe<sensor_msgs::Imu>                  ("/mavros/imu/data",                1,  &Offb_control::px4_imu_cb,this);
    quadrotor_cmd_sub =     nh.subscribe<quadrotor_msgs::PositionCommand>   ("/drone0/position_cmd",                1,  &Offb_control::quadrotor_cmd_cb,this);
    rc_in_sub =             nh.subscribe<mavros_msgs::RCIn>                 ("/mavros/rc/in",                   1,  &Offb_control::RCIn_cb,this);
    takeoff_sub =           nh.subscribe<std_msgs::Bool>                    ("/takeoff_triger",                 1,  &Offb_control::takeoff_cb,this);
    // camera_depth_sub =   nh.subscribe<sensor_msgs::Image>                   ("/elas_l/depth",                   1,  &Offb_control::depthCallback_camera, this); 

    setpoint_pub_raw =      nh.advertise<mavros_msgs::PositionTarget>       ("/mavros/setpoint_raw/local",      1);
    ego_goal_msg_pub =      nh.advertise<geometry_msgs::PoseStamped>        ("/move_base_simple/goal",          1);
    ego_cam_pose_pub =      nh.advertise<geometry_msgs::PoseStamped>        ("/camera_pose",                    1);
    local_pose_pub =        nh.advertise<nav_msgs::Odometry>                ("/mavros/odometry/out",            1);
    setpoint_raw_atti_pub = nh.advertise<mavros_msgs::AttitudeTarget>       ("/mavros/setpoint_raw/attitude",   1);
    debug_pub =             nh.advertise<nav_msgs::Odometry>                ("/debug_msg",                      1);
    triger_pub =            nh.advertise<geometry_msgs::PoseStamped>        ("/triger",                        10);

    arming_client =         nh.serviceClient<mavros_msgs::CommandBool> ("/mavros/cmd/arming");
    set_mode_client =       nh.serviceClient<mavros_msgs::SetMode>     ("/mavros/set_mode");
    reboot_FCU_srv =        nh.serviceClient<mavros_msgs::CommandLong> ("/mavros/cmd/command");

    initialize();
}

Offb_control::~Offb_control(){
    cout << "offb_control析构函数调用"  << endl;
}

void Offb_control::readParameters(){
    YAML::Node config = YAML::LoadFile("/home/khadas/demon_packages/demon_drone_ws/src/offb/src/offb_node_config.yaml");
    use_wgs84 = config["use_wgs84"].as<int>();
    num_point = config["num_point"].as<int>();
    use_mag = config["use_mag"].as<int>();
    if(use_mag == 0){
        for(int i=0;i<num_point;i++){
            point.push_back(config["point_"+ std::to_string(i)].as< std::vector<double> >());
            std::cout << "point " + std::to_string(i) + ": [ ";
            for( int i2 = 0; i2 < 3; i2++){
                std::cout << point[i][i2] << " ";
            }
            std::cout << "]" << std::endl;
        }
    }
    start_point_wgs84 = config["point_wgs84_start_position"].as< std::vector<double> >();
    if(use_mag != 0){
        for(int i=0;i<num_point;i++){
            point_wgs84.push_back(config["point_wgs84_"+ std::to_string(i)].as< std::vector<double> >());
            std::cout << "point_wgs84 " + std::to_string(i) + ": [ ";
            for( int i2 = 0; i2 < 4; i2++){
                std::cout << point_wgs84[i][i2] << " ";
            }
            std::cout << "]" << std::endl;
        }
    }
    position_threshold =        config["position_threshold"].as<float>();
    height_threshold =          config["height_threshold"].as<float>();
    time_duration =             config["time_duration"].as<float>();
    yaw_set =                   config["yaw_set"].as<float>();
    yaw_fix_threshold =         config["yaw_fix_threshold"].as<float>();
    position_orientation_set =  config["position_orientation_set"].as<float>();
    smooth =                    config["smooth"].as<float>();
    so3_control_flag =          config["so3_control_flag"].as<int>();

    std::cout << "position_threshold:" << position_threshold << 
                 "\nheight_threshold:" << height_threshold   << 
                 "\ntime_duration:"    << time_duration      << 
                 "\nyaw_set:"          << yaw_set            << 
                 "\nnum_point:"        << num_point          << 
                 "\nso3_control_flag:"  << so3_control_flag   << 
                 std::endl << std::endl;

    T_i_cam = Eigen::Isometry3d::Identity();
    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j ) {
            T_i_cam(i, j) = config["T_i_cam"][i][j].as<double>();
        }
    }
    Eigen::Isometry3d T_cam_i = T_i_cam.inverse();

    use_camIMU = config["use_camIMU"].as<int>();
    Eigen::Isometry3d T_fi_c = Eigen::Isometry3d::Identity();
    if(use_camIMU == 1){
        for(int i = 0; i < 4; ++i) {
            for(int j = 0; j < 4; ++j ) {
                T_fi_c(i, j) = config["T_fi_c"][i][j].as<double>();
            }
        }
        T_fi_ci = Eigen::Isometry3d::Identity();
        T_fi_ci = T_fi_c * T_cam_i;
        T_ci_fi = T_fi_ci.inverse();
    }

    tf::Matrix3x3 R_ci_fi;
    R_ci_fi.setValue(T_ci_fi(0, 0), T_ci_fi(0, 1), T_ci_fi(0, 2),
                     T_ci_fi(1, 0), T_ci_fi(1, 1), T_ci_fi(1, 2),
                     T_ci_fi(2, 0), T_ci_fi(2, 1), T_ci_fi(2, 2));
    R_ci_fi.getRotation(q_vins2px4);
    V_wci_ci.setZero();
}

void Offb_control::initialize(){
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_WARN("Open PX4!");
    }


    while(ros::ok() && !rcin_msg.channels.size()) {
        ros::spinOnce();
        rate.sleep();
        ROS_WARN("Open telecontroller!");
    }

    // m_local_position.lock();
    // local_position_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // local_position_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
    //                          mavros_msgs::PositionTarget::IGNORE_VY |
    //                          mavros_msgs::PositionTarget::IGNORE_VZ |
    //                          mavros_msgs::PositionTarget::IGNORE_AFX |
    //                          mavros_msgs::PositionTarget::IGNORE_AFY |
    //                          mavros_msgs::PositionTarget::IGNORE_AFZ |
    //                          mavros_msgs::PositionTarget::FORCE |
    //                         //  mavros_msgs::PositionTarget::IGNORE_YAW |
    //                          mavros_msgs::PositionTarget::IGNORE_YAW_RATE ;

    // local_position_msg.position.x = point_fixed[0][0];
    // local_position_msg.position.y = point_fixed[0][1];
    // local_position_msg.position.z = point_fixed[0][2];
    // local_position_msg.yaw = 0.0;
    // m_local_position.unlock();

    // //send a few setpoints before starting
    // rate = 20;
    // for(int i = 5; ros::ok() && i > 0; --i){
    //     setpoint_pub_raw.publish(local_position_msg);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    s = 1;
    last_request = ros::Time::now();

    // thread * interrupt_thread_      = new thread(&Offb_control::interrupt, this);
    // thread * target_set_thread_    = new thread(&Offb_control::target_set_process, this);
    thread * target_set_thread_ = new thread(&Offb_control::timer_callback, this);
    // thread * showInfo_thread = new thread(&Offb_control::showInfo, this);

    std::cout << "\nInitialize Finished! "<<std::endl;
}

void Offb_control::interrupt(void){
    while(ros::ok()){
        // send local pose
        m_local_position.lock();
        local_position_msg.header.stamp = ros::Time::now();
        // setpoint_pub_raw.publish(local_position_msg);
        m_local_position.unlock();
        
        ros::spinOnce();
        chrono::milliseconds dura(66);
        this_thread::sleep_for(dura);
    }
}

// void Offb_control::target_set_process(void){
//     while(ros::ok()){
//         if( current_state.armed && (ros::Time::now() - last_request > ros::Duration(time_duration)) ){
//             m_local_position.lock();
//             if( (abs(local_pose_infomation.pose.position.x - point_fixed[s][0]) < position_threshold) && 
//                 (abs(local_pose_infomation.pose.position.y - point_fixed[s][1]) < position_threshold) && 
//                  abs(altitude_mavros.local                 - point_fixed[s][2]) < height_threshold ){
//                     m_local_position.unlock();
//                     ++s;
//                     s = s % num_point;
//                     m_local_position.lock();
//                     ego_goal_msg.pose.position.x = point_fixed[s][0];
//                     ego_goal_msg.pose.position.y = point_fixed[s][1];
//                     ego_goal_msg.pose.position.z = point_fixed[s][2];
//                     m_local_position.unlock();
//             }
//             else m_local_position.unlock();

//             last_request = ros::Time::now();
//         }

//         ros::spinOnce();
//         chrono::milliseconds dura(50); //20Hz
//         this_thread::sleep_for(dura);
//     }
// }

void Offb_control::reboot_FCU()
{
	// https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
	mavros_msgs::CommandLong reboot_srv;
	reboot_srv.request.broadcast = false;
	reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
	reboot_srv.request.param1 = 1;	  // Reboot autopilot
	reboot_srv.request.param2 = 0;	  // Do nothing for onboard computer
	reboot_srv.request.confirmation = true;

	reboot_FCU_srv.call(reboot_srv);

	ROS_INFO("Reboot FCU");

	// if (param.print_dbg)
	// 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result, reboot_srv.response.success);
}

bool Offb_control::force_arm_disarm(bool arm)
{
	// https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
	mavros_msgs::CommandLong force_arm_disarm_srv;
	force_arm_disarm_srv.request.broadcast = false;
	force_arm_disarm_srv.request.command = 400; // MAV_CMD_COMPONENT_ARM_DISARM
	force_arm_disarm_srv.request.param1 = arm;
	force_arm_disarm_srv.request.param2 = 21196.0;	  // force
	force_arm_disarm_srv.request.confirmation = true;

	if (!(reboot_FCU_srv.call(force_arm_disarm_srv) && force_arm_disarm_srv.response.success))
	{
		if (arm)
			ROS_ERROR("ARM rejected by PX4!");
		else
			ROS_ERROR("DISARM rejected by PX4!");

		return false;
	}
}

void Offb_control::showInfo(void){
    geometry_msgs::Vector3 euler;
    tf::Matrix3x3 mat;
    while(ros::ok()){
        //固定的浮点显示
        std::cout.setf(std::ios::fixed);
        std::cout << std::setprecision(2); //设显示小数精度为n位
        //左对齐
        std::cout.setf(std::ios::left);
        // 强制显示小数点
        std::cout.setf(std::ios::showpoint);
        // 强制显示符号
        std::cout.setf(std::ios::showpos);

        cout << "Current state: " << current_state.mode << endl;
        if(takeoff_flag)
            cout << "Current mode: TAKING OFF" << endl;
        else if(land_flag)
            cout << "Current mode: LANDING" << endl;
        else if(if_takeoff && !takeoff_flag)
            cout << "Current mode: COMMAND" << endl;

        std::cout <<">>>>>>>>>>>>>>>>>>>>>>>> Drone Info [ENU Frame] <<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
        std::cout << "Drone_pose_w [x y z]: " << vins_pose.pose.pose.position.x << " [m] " << vins_pose.pose.pose.position.y << " [m] " << vins_pose.pose.pose.position.z << " [m] " << std::endl;
        std::cout << "Drone_vel    [x y z]: " << vins_pose.twist.twist.linear.x << " [m/s] " << vins_pose.twist.twist.linear.y << " [m/s] " << vins_pose.twist.twist.linear.z << " [m/s] " << std::endl;
        mat = tf::Matrix3x3(tf::Quaternion(vins_pose.pose.pose.orientation.x, vins_pose.pose.pose.orientation.y, vins_pose.pose.pose.orientation.z, vins_pose.pose.pose.orientation.w));
        mat.getRPY(euler.x, euler.y, euler.z);
        std::cout << "Drone_euler  [R P Y]: " << euler.x/M_PI*180.0 << " [deg] " << euler.y/M_PI*180.0 << " [deg] " << euler.z/M_PI*180.0 << " [deg] " << std::endl;

        std::cout <<">>>>>>>>>>>>>>>>>>>>>>>> mavros_odom Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
        // std::cout << "mavros_freq: " << mavros_odom_freq << " [Hz] " << std::endl; 
        std::cout << "mavros_pose [x y z]: " << odom_data.p(0) << " [m] " << odom_data.p(1) << " [m] " << odom_data.p(2) << " [m] " << std::endl;
        std::cout << "mavros_vel  [x y z]: " << odom_data.v(0) << " [m/s] " << odom_data.v(1) << " [m/s] " << odom_data.v(2) << " [m/s] " << std::endl;
        mat = tf::Matrix3x3(tf::Quaternion(odom_data.q.x(), odom_data.q.y(), odom_data.q.z(), odom_data.q.w()));
        mat.getRPY(euler.x, euler.y, euler.z);
        std::cout << "mavros_euler[R P Y]: " << euler.x/M_PI*180.0 << " [deg] " << euler.y/M_PI*180.0 << " [deg] " << euler.z/M_PI*180.0 << " [deg] " << std::endl;

        // std::cout <<">>>>>>>>>>>>>>>>>>>>>>>> mavros_imu Info <<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
        // mat = tf::Matrix3x3(tf::Quaternion(mavros_imu.orientation.x, mavros_imu.orientation.y, mavros_imu.orientation.z, mavros_imu.orientation.w));
        // mat.getRPY(euler.x, euler.y, euler.z);
        // std::cout << "mavros_imu[R P Y]: " << euler.x/M_PI*180.0 << " [deg] " << euler.y/M_PI*180.0 << " [deg] " << mag_yaw/M_PI*180.0 << " [deg] " << std::endl;

        // std::cout <<">>>>>>>>>>>>>>>>>>>>>>>> setpoint Info [ENU Frame] <<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
        // std::cout << "setpoint_pose [x y z yaw]: " << setpoint.position.x << " [m] " << setpoint.position.y << " [m] " << setpoint.position.z << " [m] " << setpoint.yaw/M_PI*180.0 << " [deg] " << std::endl;
        // std::cout << "setpoint_vel  [x y z yaw]: " << setpoint.velocity.x << " [m/s] " << setpoint.velocity.y << " [m/s] " << setpoint.velocity.z << " [m/s] " << setpoint.yaw_rate/M_PI*180.0 << " [deg/s] " << std::endl;
        // std::cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> END <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
        std::cout << std::endl;
        ros::spinOnce();
        chrono::milliseconds dura(100);
        this_thread::sleep_for(dura);
    }
}

void Offb_control::timer_callback(void){
    LinearControl controller(param);
    ros::Rate r(param.ctrl_freq_max);
    last_time = ros::Time::now();

    int last_rc = 0;

    while(ros::ok()){
        r.sleep();
        ros::spinOnce();
        now_time = ros::Time::now();

        m_controller.lock();
        m_px4_imu.lock();

        controller.estimateThrustModel(imu_data.a,param);

        //HOVER
        if(rcin_msg.channels[6] < 1800){
            controller.resetThrustMapping();

            des.p.x() = odom_data.p.x();
            des.p.y() = odom_data.p.y();
            des.p.z() = odom_data.p.z();
            des.v.setZero();
            des.a.setZero();
            des.j.setZero();

            Eigen::Quaterniond q(odom_data.q.w(), odom_data.q.x(), odom_data.q.y(), odom_data.q.z());
            des.yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
            des.yaw_rate = 0.0;
        }

        //AUTO_TAKEOFF
        constexpr double MOTORS_SPEEDUP_TIME = 3.0;
        static bool motor_speedup_complete = false;
		// if (rcin_msg.channels[6] > 1800 && takeoff_triger && !if_takeoff)
        if(rcin_msg.channels[6] > 1800 && last_rc < 1800) {
            takeoff_flag = true;
            last_request = ros::Time::now();
        }
		if (takeoff_flag && rcin_msg.channels[6] > 1800)
		{
            if(!current_state.armed && (ros::Time::now() - last_request).toSec() > 1.5) {
                des.a.z() = -9.8;
                ROS_INFO("Vehicle arming");
                force_arm_disarm(true);
                last_request = ros::Time::now();
            }
            else if (!motor_speedup_complete && current_state.armed && (ros::Time::now() - last_request).toSec() < MOTORS_SPEEDUP_TIME){
                double delta_t = (ros::Time::now() - last_request).toSec();
                double des_a_z = exp((delta_t - MOTORS_SPEEDUP_TIME) * 6.0) * 9.7 - 9.7; // Parameters 6.0 and 7.0 are just heuristic values which result in a saticfactory curve.
                if (des_a_z > 0.1)
                {
                    ROS_ERROR("des_a_z > 0.1!, des_a_z=%f", des_a_z);
                    des_a_z = 0.0;
                }
                des.p.z() = odom_data.p.z();
                des.v = Eigen::Vector3d::Zero();
                des.a = Eigen::Vector3d(0, 0, des_a_z);
                des.j = Eigen::Vector3d::Zero();
            }
            else if(!if_takeoff && current_state.armed) {
                if(!motor_speedup_complete) {
                    last_request = ros::Time::now();
                    motor_speedup_complete = true;
                }
                des.p.z() = odom_data.p.z() + param.takeoff_land.speed * (ros::Time::now() - last_request).toSec();
                des.v = Eigen::Vector3d(0, 0, param.takeoff_land.speed);
                des.a = Eigen::Vector3d::Zero();
                des.j = Eigen::Vector3d::Zero();
            }

            if (des.p.z() >= param.takeoff_land.height) // reach the desired height
            {
                des.v.setZero();
                des.a.setZero();
                des.j.setZero();
                if_takeoff = true;
                takeoff_flag = false;
                if_land = false;
            }
        }

        //AUTO_LAND
        static bool last_land_flag = false;
        static double start_land_height = 0.0;
        land_flag = get_planning_msg && (ros::Time::now() - planning_timestamp).toSec() > 3.0;
        if(land_flag && !last_land_flag) {
            last_request = ros::Time::now();
            start_land_height = odom_data.p.z();
        }
        last_land_flag = land_flag;
        if(current_state.armed && land_flag) {
            des.p.z() = start_land_height - param.takeoff_land.speed * (ros::Time::now() - last_request).toSec();
            des.v = Eigen::Vector3d(0, 0, -param.takeoff_land.speed);
            des.a = Eigen::Vector3d::Zero();
            des.j = Eigen::Vector3d::Zero();

            // land_detector parameters
            constexpr double POSITION_DEVIATION_C = -0.5; // Constraint 1: target position below real position for POSITION_DEVIATION_C meters.
            constexpr double VELOCITY_THR_C = 0.1;		  // Constraint 2: velocity below VELOCITY_MIN_C m/s.
            constexpr double TIME_KEEP_C = 3.0;			  // Constraint 3: Time(s) the Constraint 1&2 need to keep.

            static ros::Time time_C12_reached; // time_Constraints12_reached
            static bool is_last_C12_satisfy = false;

            bool C12_satisfy = (des.p(2) - odom_data.p(2)) < POSITION_DEVIATION_C && odom_data.v.norm() < VELOCITY_THR_C;
            if (C12_satisfy && !is_last_C12_satisfy)
            {
                time_C12_reached = ros::Time::now();
            }
            else if (C12_satisfy && is_last_C12_satisfy)
            {
                if ((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP_C) //Constraint 3 reached
                {
                    if_land = true;
                    get_planning_msg = false;
                    force_arm_disarm(false);
                }
            }
            is_last_C12_satisfy = C12_satisfy;
        }

        Eigen::Vector3d des_acc(0.0, 0.0, 0.0);//for debug
        controller.calculateControl(des, odom_data, imu_data, u, des_acc);

        mavros_msgs::AttitudeTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = std::string("FCU");
        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        msg.orientation.x = u.q.x();
        msg.orientation.y = u.q.y();
        msg.orientation.z = u.q.z();
        msg.orientation.w = u.q.w();
        msg.thrust = u.thrust;

        setpoint_raw_atti_pub.publish(msg);

        nav_msgs::Odometry debug_msg;
        debug_msg.header.stamp = msg.header.stamp;
        debug_msg.pose.pose.position.x = des.p.x();
        debug_msg.pose.pose.position.y = des.p.y();
        debug_msg.pose.pose.position.z = des.p.z();
        debug_msg.twist.twist.linear.x = des.v.x();
        debug_msg.twist.twist.linear.y = des.v.y();
        debug_msg.twist.twist.linear.z = des.v.z();
        // debug_msg.pose.pose.orientation.w = des.q.w();
        // debug_msg.pose.pose.orientation.x = des.q.x();
        // debug_msg.pose.pose.orientation.y = des.q.y();
        // debug_msg.pose.pose.orientation.z = des.q.z();
        //借用该变量来查看油门和期望加速度
        debug_msg.pose.pose.orientation.w = u.thrust;
        debug_msg.pose.pose.orientation.x = des_acc.x();
        debug_msg.pose.pose.orientation.y = des_acc.y();
        debug_msg.pose.pose.orientation.z = des_acc.z();
        debug_pub.publish(debug_msg);
        // printf("%6.2f %6.2f\n", 
        //         u.thrust, atan2(2 * (u.q.x()*u.q.y() + u.q.w()*u.q.z()), u.q.w()*u.q.w() + u.q.x()*u.q.x() - u.q.y()*u.q.y() - u.q.z()*u.q.z()) * 57.3);

        //发布triger，发送目标追踪开始信号
        triger_msg.header.stamp = ros::Time::now();
        if(rcin_msg.channels[9] > 1900) {
            triger_pub.publish(triger_msg);
        }

        m_px4_imu.unlock();
        m_controller.unlock();
        last_time = now_time;
        last_rc = rcin_msg.channels[6];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    Offb_control offb_control;
    ros::spin();

    ROS_INFO("offb_node closed.\n");
    return 0;
}

