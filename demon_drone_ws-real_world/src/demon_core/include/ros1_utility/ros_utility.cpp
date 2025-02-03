#include "ros_utility.hpp"

void Ros_Utility_t::spin(int argc, char** argv, bool if_pub_stereo, bool if_pub_demon, bool if_pub_planner){
    ros::init(argc, argv, "demon_core_node");
    ros::MultiThreadedSpinner spinner(0);
    ros::NodeHandle nh("~");

    if(if_pub_stereo){
        image_transport::ImageTransport it(nh);  //  类似ROS句柄
        lImage_pub      = it.advertise("/stereo/left/image_raw", 1);
        rImage_pub      = it.advertise("/stereo/right/image_raw", 1);
        lImage_rect_pub = it.advertise("/stereo/left/image_rect", 1);
        rImage_rect_pub = it.advertise("/stereo/right/image_rect", 1);
        depth_pub       = it.advertise("/stereo/depth", 1);
        lCam_info_pub   = nh.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 1);
        rCam_info_pub   = nh.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 1);
    }

    if(if_pub_demon){
        imu_pub         = nh.advertise<sensor_msgs::Imu>            ("/demon/imu/data", 1);
        odom_pub        = nh.advertise<nav_msgs::Odometry>          ("/demon/odom", 10);
        camPos_pub      = nh.advertise<geometry_msgs::PoseStamped>  ("/demon/stereo/pos", 10);
        pointCloud_pub  = nh.advertise<sensor_msgs::PointCloud2>    ("/demon/point_cloud", 10);
        vis_drone_pub   = nh.advertise<visualization_msgs::Marker>  ("/demon/visualization/drone_model", 10, true);
    }

    if(if_pub_planner){
        path_pub[0]     = nh.advertise<visualization_msgs::Marker>("/SGP/planner/a_star_path"        , 10);  // 0
        path_pub[1]     = nh.advertise<visualization_msgs::Marker>("/SGP/planner/low_mpc_traj"       , 10);  // 1
        path_pub[2]     = nh.advertise<visualization_msgs::Marker>("/SGP/planner/local_goal"         , 10);  // 2
        path_pub[3]     = nh.advertise<visualization_msgs::Marker>("/SGP/planner/high_mpcc_traj"     , 10);  // 3
        path_pub[4]     = nh.advertise<visualization_msgs::Marker>("/SGP/planner/high_mpcc_ref_traj" , 10);  // 4
        path_pub[5]     = nh.advertise<visualization_msgs::Marker>("/SGP/planner/pos_cmd_vis"        , 10);  // 5
        path_pub[6]     = nh.advertise<visualization_msgs::Marker>("/SGP/planner/travel_traj"        , 10);  // 6
    }
    ready = true;
    spinner.spin();
}


void Ros_Utility_t::pub_Image(double img_t, cv::Mat &lFrame, cv::Mat &rFrame){
    if(!ready) return;
    if(lImage_pub.getNumSubscribers() == 0 && rImage_pub.getNumSubscribers() == 0) return;
    static sensor_msgs::Image left_img, right_img;
    static std_msgs::Header header;

    header.frame_id = "camera";
    header.stamp.sec = int(img_t);
    header.stamp.nsec = (img_t - header.stamp.sec) * 1e9;
    cv_bridge::CvImage(header, "bgr8", lFrame).toImageMsg(left_img);
    cv_bridge::CvImage(header, "bgr8", rFrame).toImageMsg(right_img);

    lImage_pub.publish(left_img);
    rImage_pub.publish(right_img);
}

void Ros_Utility_t::pub_image_rect(double img_t, cv::Mat &lFrame, cv::Mat &rFrame){
    if(!ready) return;
    if(lImage_rect_pub.getNumSubscribers() == 0 && rImage_rect_pub.getNumSubscribers() == 0) return;
    static sensor_msgs::Image left_img, right_img;
    static std_msgs::Header header;

    header.frame_id = "camera";
    header.stamp.sec = int(img_t);
    header.stamp.nsec = (img_t - header.stamp.sec) * 1e9;
    cv_bridge::CvImage(header, "mono8", lFrame).toImageMsg(left_img);
    cv_bridge::CvImage(header, "mono8", rFrame).toImageMsg(right_img);

    lImage_rect_pub.publish(left_img);
    rImage_rect_pub.publish(right_img);

    // static bool if_init = false;
    // if(!if_init){
    //     if_init = true;
    //     lCam_info.width  = rCam_info.width  = lFrame.cols;
    //     lCam_info.height = rCam_info.height = lFrame.row;
    //     lCam_info.distortion_model = "plumb_bob";
    //     lCam_info.header.frame_id = rCam_info.header.frame_id = "camera";  //frame_id为camera，也就是相机名字
    //     lCam_info.D = {stereoMatch.D1.at<double>(0,0), stereoMatch.D1.at<double>(1,0), stereoMatch.D1.at<double>(2,0), 
    //                    stereoMatch.D1.at<double>(3,0), stereoMatch.D1.at<double>(4,0)};  // distortion_coefficients k1,k2,p1,p2,k3=0
    //     lCam_info.K = {  // camera_matrix
    //         stereoMatch.K1.at<double>(0,0), stereoMatch.K1.at<double>(0,1), stereoMatch.K1.at<double>(0,2), 
    //         stereoMatch.K1.at<double>(1,0), stereoMatch.K1.at<double>(1,1), stereoMatch.K1.at<double>(1,2), 
    //         stereoMatch.K1.at<double>(2,0), stereoMatch.K1.at<double>(2,1), stereoMatch.K1.at<double>(2,2)
    //     };
    //     lCam_info.R = { // rectification_matrix
    //         stereoMatch.R1.at<double>(0,0), stereoMatch.R1.at<double>(0,1), stereoMatch.R1.at<double>(0,2), 
    //         stereoMatch.R1.at<double>(1,0), stereoMatch.R1.at<double>(1,1), stereoMatch.R1.at<double>(1,2), 
    //         stereoMatch.R1.at<double>(2,0), stereoMatch.R1.at<doubheightle>(2,1), stereoMatch.R1.at<double>(2,2)
    //     };     
    //     lCam_info.P = { // projection_matrix
    //         stereoMatch.P1.at<double>(0,0), stereoMatch.P1.at<double>(0,1), stereoMatch.P1.at<double>(0,2), stereoMatch.P1.at<double>(0,3), 
    //         stereoMatch.P1.at<double>(1,0), stereoMatch.P1.at<double>(1,1), stereoMatch.P1.at<double>(1,2), stereoMatch.P1.at<double>(1,3), 
    //         stereoMatch.P1.at<double>(2,0), stereoMatch.P1.at<double>(2,1), stereoMatch.P1.at<double>(2,2), stereoMatch.P1.at<double>(2,3)
    //     };
    //     lCam_info.binning_x = 0;
    //     lCam_info.binning_y = 0;

    //     rCam_info.D = {stereoMatch.D2.at<double>(0,0), stereoMatch.D2.at<double>(1,0), stereoMatch.D2.at<double>(2,0), 
    //                    stereoMatch.D2.at<double>(3,0), stereoMatch.D2.at<double>(4,0)};  // distortion_coefficients k1,k2,p1,p2,k3=0
    //     rCam_info.K = {  // camera_matrix
    //         stereoMatch.K2.at<double>(0,0), stereoMatch.K2.at<double>(0,1), stereoMatch.K2.at<double>(0,2), 
    //         stereoMatch.K2.at<double>(1,0), stereoMatch.K2.at<double>(1,1), stereoMatch.K2.at<double>(1,2), 
    //         stereoMatch.K2.at<double>(2,0), stereoMatch.K2.at<double>(2,1), stereoMatch.K2.at<double>(2,2)
    //     };
    //     rCam_info.R = { // rectification_matrix
    //         stereoMatch.R2.at<double>(0,0), stereoMatch.R2.at<double>(0,1), stereoMatch.R2.at<double>(0,2), 
    //         stereoMatch.R2.at<double>(1,0), stereoMatch.R2.at<double>(1,1), stereoMatch.R2.at<double>(1,2), 
    //         stereoMatch.R2.at<double>(2,0), stereoMatch.R2.at<double>(2,1), stereoMatch.R2.at<double>(2,2)
    //     };     
    //     rCam_info.P = { // projection_matrix
    //         stereoMatch.P2.at<double>(0,0), stereoMatch.P2.at<double>(0,1), stereoMatch.P2.at<double>(0,2), stereoMatch.P2.at<double>(0,3), 
    //         stereoMatch.P2.at<double>(1,0), stereoMatch.P2.at<double>(1,1), stereoMatch.P2.at<double>(1,2), stereoMatch.P2.at<double>(1,3), 
    //         stereoMatch.P2.at<double>(2,0), stereoMatch.P2.at<double>(2,1), stereoMatch.P2.at<double>(2,2), stereoMatch.P2.at<double>(2,3)
    //     };
    //     rCam_info.binning_x = 0;
    //     rCam_info.binning_y = 0;
    // }

    // lCam_info.header.stamp = rCam_info.header.stamp = header.stamp;
    // lCam_info_pub.publish(lCam_info);
    // rCam_info_pub.publish(rCam_info);
}

void Ros_Utility_t::pub_imu(double imu_t, Eigen::Vector3d &acc, Eigen::Vector3d &gyr){
    if(!ready) return;
    static sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "imu";
    imu_msg.header.stamp.sec = int(imu_t);
    imu_msg.header.stamp.nsec = (imu_t - imu_msg.header.stamp.sec) * 1e9;
    imu_msg.linear_acceleration.x = acc[0];
    imu_msg.linear_acceleration.y = acc[1];
    imu_msg.linear_acceleration.z = acc[2];
    imu_msg.angular_velocity.x = gyr[0];
    imu_msg.angular_velocity.y = gyr[1];
    imu_msg.angular_velocity.z = gyr[2];

    imu_pub.publish(imu_msg);  
}

void Ros_Utility_t::pub_odom(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Quaterniond &quad){
    if(!ready) return;
    // 发布里程计
    static nav_msgs::Odometry odom_msg; 

    odom_msg.header.frame_id = "world";
    odom_msg.header.stamp.sec = int(t);
    odom_msg.header.stamp.nsec = (t - odom_msg.header.stamp.sec) * 1e9;
    odom_msg.pose.pose.position.x = pos[0];
    odom_msg.pose.pose.position.y = pos[1];
    odom_msg.pose.pose.position.z = pos[2];
    odom_msg.pose.pose.orientation.x = quad.x();
    odom_msg.pose.pose.orientation.y = quad.y();
    odom_msg.pose.pose.orientation.z = quad.z();
    odom_msg.pose.pose.orientation.w = quad.w();
    odom_msg.twist.twist.linear.x = vel[0];
    odom_msg.twist.twist.linear.y = vel[1];
    odom_msg.twist.twist.linear.z = vel[2];

    odom_pub.publish(odom_msg);

    /* 发布可视化飞机模型 */
    static visualization_msgs::Marker planeMarker;
    planeMarker.id = 0;
    planeMarker.header = odom_msg.header;
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
    tf::Quaternion q2(quad.x(), quad.y(), quad.z(), quad.w());
    tf::Quaternion new_q = q1 * q2;

    geometry_msgs::Vector3 euler;
    tf::Matrix3x3 plane_mat(tf::Quaternion(new_q.x(), new_q.y(), new_q.z(), new_q.w()));
    plane_mat.getRPY(euler.x, euler.y, euler.z);  // 获得欧拉角
    geometry_msgs::Quaternion transform_q = tf::createQuaternionMsgFromRollPitchYaw(-euler.x-M_PI/2 , euler.y+M_PI, euler.z+M_PI); //  + M_PI/2    

    planeMarker.pose.orientation.w = transform_q.w;
    planeMarker.pose.orientation.x = transform_q.x;
    planeMarker.pose.orientation.y = transform_q.y;
    planeMarker.pose.orientation.z = transform_q.z;
    planeMarker.pose.position.x = pos[0];
    planeMarker.pose.position.y = pos[1];
    planeMarker.pose.position.z = pos[2];
    planeMarker.mesh_resource = std::string("package://demon_core/model/Plane.fbx");   
    vis_drone_pub.publish(planeMarker);    

    // 发布tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));
    transform.setRotation(tf::Quaternion(quad.x(), quad.y(), quad.z(), quad.w()));
    br.sendTransform(tf::StampedTransform(transform, odom_msg.header.stamp, "world", "body"));
}

void Ros_Utility_t::pub_depth(double img_t, cv::Mat &depth_img){
    if(!ready) return;
    static sensor_msgs::Image depth_msg;
    static std_msgs::Header header;

    header.frame_id = "camera";
    header.stamp.sec = int(img_t);
    header.stamp.nsec = (img_t - header.stamp.sec) * 1e9;
    cv_bridge::CvImage(header, "mono8", depth_img).toImageMsg(depth_msg);
    depth_pub.publish(depth_msg);    
}

void Ros_Utility_t::pub_cam_pos(double t, Eigen::Vector3d &pos, Eigen::Quaterniond &quad){
    if(!ready) return;
    static geometry_msgs::PoseStamped pos_msg;

    pos_msg.header.frame_id = "world";
    pos_msg.header.stamp.sec = int(t);
    pos_msg.header.stamp.nsec = (t - pos_msg.header.stamp.sec) * 1e9;
    pos_msg.pose.position.x = pos[0];
    pos_msg.pose.position.y = pos[1];
    pos_msg.pose.position.z = pos[2];
    pos_msg.pose.orientation.x = quad.x();
    pos_msg.pose.orientation.y = quad.y();
    pos_msg.pose.orientation.z = quad.z();
    pos_msg.pose.orientation.w = quad.w();

    camPos_pub.publish(pos_msg);
}
void Ros_Utility_t::pub_point_cloud(pcl::PointCloud<pcl::PointXYZ> &cloud){
    if(!ready) return;
    static sensor_msgs::PointCloud2 pointCloud_msg;
    pcl::toROSMsg(cloud, pointCloud_msg);
    pointCloud_pub.publish(pointCloud_msg);
}

void Ros_Utility_t::displayLineStrip(double t, const std::vector<Eigen::Vector3d>& list, double line_width, const Eigen::Vector4d& color, int id, int pub_id){
    visualization_msgs::Marker mk;
    mk.header.frame_id      = "world";
    mk.header.stamp.sec     = int(t);
    mk.header.stamp.nsec = (t - mk.header.stamp.sec) * 1e9;
    mk.type                 = visualization_msgs::Marker::LINE_STRIP;
    mk.action               = visualization_msgs::Marker::DELETE;
    mk.id                   = id;

    mk.action             = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);
    mk.scale.x = line_width;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); ++i) {
        pt.x = list[i](0) + origin_pos[0];
        pt.y = list[i](1) + origin_pos[1];
        pt.z = list[i](2) + origin_pos[2];
        mk.points.push_back(pt);
    }
    path_pub[pub_id].publish(mk);
}

void Ros_Utility_t::displaySphereList(double t, const std::vector<Eigen::Vector3d>& list, double resolution, const Eigen::Vector4d& color, int id, int pub_id) {
    visualization_msgs::Marker mk;
    mk.header.frame_id      = "world";
    mk.header.stamp.sec     = int(t);
    mk.header.stamp.nsec = (t - mk.header.stamp.sec) * 1e9;
    mk.type            = visualization_msgs::Marker::SPHERE_LIST;
    mk.action          = visualization_msgs::Marker::DELETE;
    mk.id              = id;

    mk.action             = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++) {
        pt.x = list[i](0) + origin_pos[0];
        pt.y = list[i](1) + origin_pos[1];
        pt.z = list[i](2) + origin_pos[2];
        mk.points.push_back(pt);
    }
    path_pub[pub_id].publish(mk);
}


void Ros_Utility_t::drawAstar(double t, const std::vector<Eigen::Vector3d>& path, double line_width, const Eigen::Vector4d& color){
    if(!ready) return;
    displayLineStrip(t, path, line_width, color, 0, 0);
}

void Ros_Utility_t::drawLowMpcTraj(double t, const std::vector<Eigen::Vector3d>& traj, double resolution, const Eigen::Vector4d& color) {
    if(!ready) return;
    displaySphereList(t, traj, resolution, color, 1, 1);
}

void Ros_Utility_t::drawLocalGoal(double t, Eigen::Vector3d local_goal, double resolution, const Eigen::Vector4d& color) {
    if(!ready) return;
    std::vector<Eigen::Vector3d> local_goal_vec = { local_goal };
    displaySphereList(t, local_goal_vec, resolution, color, 2, 2);
}

void Ros_Utility_t::drawHighMpccTraj(double t, const std::vector<Eigen::Vector3d>& traj, double resolution, const Eigen::Vector4d& color) {
    if(!ready) return;
    displaySphereList(t, traj, resolution, color, 3, 3);
}

void Ros_Utility_t::drawHighMpccRefTraj(double t, const std::vector<Eigen::Vector3d>& traj, double resolution, const Eigen::Vector4d& color) {
    if(!ready) return;
    displaySphereList(t, traj, resolution, color, 4, 4);
}

void Ros_Utility_t::drawCmd(double t, const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id, const Eigen::Vector4d& color){
    if(!ready) return;
    static visualization_msgs::Marker mk;
    mk.header.frame_id      = "world";
    mk.header.stamp.sec     = int(t);
    mk.header.stamp.nsec = (t - mk.header.stamp.sec) * 1e9;
    mk.id = id;
    mk.type = visualization_msgs::Marker::ARROW;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.1;
    mk.scale.y = 0.2;
    mk.scale.z = 0.3;

    geometry_msgs::Point pt;
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);
    mk.points.push_back(pt);

    pt.x = pos(0) + vec(0);
    pt.y = pos(1) + vec(1);
    pt.z = pos(2) + vec(2);
    mk.points.push_back(pt);

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    path_pub[5].publish(mk);   
}

void Ros_Utility_t::drawDroneTraj(double t, const Eigen::Vector3d &pos, const Eigen::Vector3d &vel){
    if(!ready) return;
    static visualization_msgs::Marker mk;
    mk.header.frame_id      = "world";
    mk.header.stamp.sec     = int(t);
    mk.header.stamp.nsec = (t - mk.header.stamp.sec) * 1e9;
    mk.type = visualization_msgs::Marker::LINE_STRIP;
    mk.id = 6;
    mk.scale.x = 0.15;  // 线宽
    // 设置颜色
    double v_norm = vel.norm();
    double color_num;
    double v_min_ = 0;
    double v_max_ = 3;
    if(v_norm < v_min_)         color_num = 0.0;
    else if(v_norm > v_max_)    color_num = 1.0;
    else                        color_num = (v_norm - v_min_) / (v_max_ - v_min_);
    std_msgs::ColorRGBA color;  
    color.r = color_num>0.5 ? 2*color_num-1 : 0.0;
    color.g = color_num>0.5 ? -2*color_num+2 : 2*color_num;
    color.b = color_num>0.5 ? 0.0 : -2*color_num+1;
    color.a = 1.0;
    mk.colors.push_back(color);

    geometry_msgs::Point pt;
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);    
    mk.points.push_back(pt);
    if(mk.points.size() >= 2000) mk.points.erase(mk.points.begin());
    path_pub[6].publish(mk);
}

void Ros_Utility_t::set_origin_pos(const Eigen::Vector3d &origin_pos_){
    if(!ready) return;
    origin_pos = origin_pos_;
}