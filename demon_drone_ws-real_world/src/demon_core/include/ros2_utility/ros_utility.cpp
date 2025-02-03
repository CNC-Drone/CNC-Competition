#include "ros_utility.hpp"

void Ros_Utility_t::spin(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("demon_core_node");
    if(if_pub_stereo){
        lImage_pub      = node->create_publisher<sensor_msgs::msg::Image>("/stereo/left/image_raw"  , 10);
        rImage_pub      = node->create_publisher<sensor_msgs::msg::Image>("/stereo/right/image_raw" , 10);
        depth_pub       = node->create_publisher<sensor_msgs::msg::Image>("/stereo/depth"           , 10);
    }

    if(if_pub_demon){
        imu_pub         = node->create_publisher<sensor_msgs::msg::Imu>  ("/demon/imu/data"         , 10);
        gps_pub         = node->create_publisher<sensor_msgs::msg::NavSatFix>("/demon/gps"          , 10);
        odom_pub        = node->create_publisher<nav_msgs::msg::Odometry>("/demon/odom"             , 10);
        vis_drone_pub   = node->create_publisher<visualization_msgs::msg::Marker>("/demon/drone/marker"             , 10);
    }

    if(if_pub_planner){
        pointCloud_pub  = node->create_publisher<sensor_msgs::msg::PointCloud2>  ("/SGP/occupy/point_cloud"         , 10);
        path_pub[0]     = node->create_publisher<visualization_msgs::msg::Marker>("/SGP/planner/a_star_path"        , 10);  // 0
        path_pub[1]     = node->create_publisher<visualization_msgs::msg::Marker>("/SGP/planner/low_mpc_traj"       , 10);  // 1
        path_pub[2]     = node->create_publisher<visualization_msgs::msg::Marker>("/SGP/planner/local_goal"         , 10);  // 2
        path_pub[3]     = node->create_publisher<visualization_msgs::msg::Marker>("/SGP/planner/high_mpcc_traj"     , 10);  // 3
        path_pub[4]     = node->create_publisher<visualization_msgs::msg::Marker>("/SGP/planner/high_mpcc_ref_traj" , 10);  // 4
        path_pub[5]     = node->create_publisher<visualization_msgs::msg::Marker>("/SGP/planner/pos_cmd_vis"        , 10);  // 5
        path_pub[6]     = node->create_publisher<visualization_msgs::msg::Marker>("/SGP/planner/travel_traj"        , 10);  // 6
    }

    tf_wb_pub = std::make_unique<tf2_ros::TransformBroadcaster>(*node);

    ready = true;
    rclcpp::spin(node);
    rclcpp::shutdown();
}

void Ros_Utility_t::pub_Image(double img_t, cv::Mat &lFrame, cv::Mat &rFrame){
    if(!ready) return;
    static sensor_msgs::msg::Image left_img, right_img;
    static std_msgs::msg::Header header;
    header.frame_id = "camera";
    header.stamp.sec = int(img_t);
    header.stamp.nanosec = (img_t - header.stamp.sec) * 1e9;
    cv_bridge::CvImage(header, "bgr8", lFrame).toImageMsg(left_img);    // 原始为bgr8格式，灰度为mono8
    cv_bridge::CvImage(header, "bgr8", rFrame).toImageMsg(right_img);
    lImage_pub->publish(left_img);
    rImage_pub->publish(right_img);
}

void Ros_Utility_t::pub_imu(double imu_t, Eigen::Vector3d &acc, Eigen::Vector3d &gyr){
    if(!ready) return;
    static sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "imu";
    imu_msg.header.stamp.sec = int(imu_t);
    imu_msg.header.stamp.nanosec = (imu_t - imu_msg.header.stamp.sec) * 1e9;
    imu_msg.linear_acceleration.x = acc[0];
    imu_msg.linear_acceleration.y = acc[1];
    imu_msg.linear_acceleration.z = acc[2];
    imu_msg.angular_velocity.x = gyr[0];
    imu_msg.angular_velocity.y = gyr[1];
    imu_msg.angular_velocity.z = gyr[2];
    imu_pub->publish(imu_msg); 
}

void Ros_Utility_t::pub_gps(double gps_t, double lat, double lon, double alt, int satellites_visible){
    if(!ready) return;
    static sensor_msgs::msg::NavSatFix ros_msg;
    ros_msg.header.frame_id = "GPS";
    ros_msg.header.stamp.sec = int(gps_t);
    ros_msg.header.stamp.nanosec = (gps_t - ros_msg.header.stamp.sec) * 1e9;
    ros_msg.status.service = satellites_visible;
    ros_msg.latitude = lat;
    ros_msg.longitude = lon;
    ros_msg.altitude = alt;
    gps_pub->publish(ros_msg);
}

void Ros_Utility_t::pub_odom(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Quaterniond &quad){
    if(!ready) return;
    static nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "world";
    odom_msg.header.stamp.sec = int(t);
    odom_msg.header.stamp.nanosec = (t - odom_msg.header.stamp.sec) * 1e9;
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
    odom_pub->publish(odom_msg);

    // 发布tf
    static geometry_msgs::msg::TransformStamped tf_wb;
    tf_wb.header.stamp      = odom_msg.header.stamp;
    tf_wb.header.frame_id   = "world";
    tf_wb.child_frame_id    = "body";
    tf_wb.transform.translation.x = pos[0];
    tf_wb.transform.translation.y = pos[1];
    tf_wb.transform.translation.z = pos[2];
    tf_wb.transform.rotation.x = quad.x();
    tf_wb.transform.rotation.y = quad.y();
    tf_wb.transform.rotation.z = quad.z();
    tf_wb.transform.rotation.w = quad.w();
    tf_wb_pub->sendTransform(tf_wb);

    /* 发布可视化飞机模型 */
    static visualization_msgs::msg::Marker planeMarker;
    planeMarker.id      = 0;
    planeMarker.header  = odom_msg.header;
    planeMarker.action  = visualization_msgs::msg::Marker::ADD;
    planeMarker.type    = visualization_msgs::msg::Marker::MESH_RESOURCE;
    planeMarker.ns      = "demon_drone";
    planeMarker.mesh_use_embedded_materials = true;
    planeMarker.pose.position = odom_msg.pose.pose.position;

    Eigen::Matrix3d rotation_matrix1;
    rotation_matrix1 =  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond rota_quad = Eigen::Quaterniond(rotation_matrix1);
    Eigen::Quaterniond drone_quad = quad * rota_quad;
    planeMarker.pose.orientation.x = drone_quad.x();
    planeMarker.pose.orientation.x = drone_quad.y();
    planeMarker.pose.orientation.x = drone_quad.z();
    planeMarker.pose.orientation.x = drone_quad.w();
    // planeMarker.color.r = 0.0;
    // planeMarker.color.g = 0.0;
    // planeMarker.color.b = 0.0;
    // planeMarker.color.a = 0.0;
    planeMarker.scale.x = 0.002;
    planeMarker.scale.y = 0.002;
    planeMarker.scale.z = 0.002; 
    planeMarker.mesh_resource = "package://demon_core/meshes/demon.stl";
    vis_drone_pub->publish(planeMarker);
}

void Ros_Utility_t::pub_depth(double img_t, cv::Mat &depth_img){
    if(!ready) return;
    static sensor_msgs::msg::Image depth_msg;
    static std_msgs::msg::Header header;

    header.frame_id = "camera";
    header.stamp.sec = int(img_t);
    header.stamp.nanosec = (img_t - header.stamp.sec) * 1e9;
    cv_bridge::CvImage(header, "mono8", depth_img).toImageMsg(depth_msg);
    depth_pub->publish(depth_msg);    
}
void Ros_Utility_t::pub_image_rect(double img_t, cv::Mat &lFrame, cv::Mat &rFrame){}
void Ros_Utility_t::pub_cam_pos(double t, Eigen::Vector3d &pos, Eigen::Quaterniond &quad){}

void Ros_Utility_t::pub_point_cloud(pcl::PointCloud<pcl::PointXYZ> &cloud){
    if(!ready) return;
    static sensor_msgs::msg::PointCloud2 pointCloud_msg;
    pcl::toROSMsg(cloud, pointCloud_msg);
    pointCloud_pub->publish(pointCloud_msg);
}

void Ros_Utility_t::displayLineStrip(double t, const std::vector<Eigen::Vector3d>& list, double line_width, const Eigen::Vector4d& color, int id, int pub_id){
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id      = "world";
    mk.header.stamp.sec     = int(t);
    mk.header.stamp.nanosec = (t - mk.header.stamp.sec) * 1e9;
    mk.type                 = visualization_msgs::msg::Marker::LINE_STRIP;
    mk.action               = visualization_msgs::msg::Marker::DELETE;
    mk.id                   = id;
    // path_pub[pub_id]->publish(mk);

    mk.action             = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);
    mk.scale.x = line_width;

    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(list.size()); ++i) {
        pt.x = list[i](0) + origin_pos[0];
        pt.y = list[i](1) + origin_pos[1];
        pt.z = list[i](2) + origin_pos[2];
        mk.points.push_back(pt);
    }
    path_pub[pub_id]->publish(mk);
}

void Ros_Utility_t::displaySphereList(double t, const std::vector<Eigen::Vector3d>& list, double resolution, const Eigen::Vector4d& color, int id, int pub_id) {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id      = "world";
    mk.header.stamp.sec     = int(t);
    mk.header.stamp.nanosec = (t - mk.header.stamp.sec) * 1e9;
    mk.type            = visualization_msgs::msg::Marker::SPHERE_LIST;
    mk.action          = visualization_msgs::msg::Marker::DELETE;
    mk.id              = id;
    // path_pub[pub_id]->publish(mk);

    mk.action             = visualization_msgs::msg::Marker::ADD;
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

    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(list.size()); i++) {
        pt.x = list[i](0) + origin_pos[0];
        pt.y = list[i](1) + origin_pos[1];
        pt.z = list[i](2) + origin_pos[2];
        mk.points.push_back(pt);
    }
    path_pub[pub_id]->publish(mk);
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
    static visualization_msgs::msg::Marker mk;
    mk.header.frame_id      = "world";
    mk.header.stamp.sec     = int(t);
    mk.header.stamp.nanosec = (t - mk.header.stamp.sec) * 1e9;
    mk.id = id;
    mk.type = visualization_msgs::msg::Marker::ARROW;
    mk.action = visualization_msgs::msg::Marker::ADD;

    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.1;
    mk.scale.y = 0.2;
    mk.scale.z = 0.3;

    geometry_msgs::msg::Point pt;
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

    path_pub[5]->publish(mk);   
}

void Ros_Utility_t::drawDroneTraj(double t, const Eigen::Vector3d &pos, const Eigen::Vector3d &vel){
    if(!ready) return;
    static visualization_msgs::msg::Marker mk;
    mk.header.frame_id      = "world";
    mk.header.stamp.sec     = int(t);
    mk.header.stamp.nanosec = (t - mk.header.stamp.sec) * 1e9;
    mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
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
    std_msgs::msg::ColorRGBA color;  
    color.r = color_num>0.5 ? 2*color_num-1 : 0.0;
    color.g = color_num>0.5 ? -2*color_num+2 : 2*color_num;
    color.b = color_num>0.5 ? 0.0 : -2*color_num+1;
    color.a = 1.0;
    mk.colors.push_back(color);

    geometry_msgs::msg::Point pt;
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);    
    mk.points.push_back(pt);
    if(mk.points.size() >= 2000) mk.points.erase(mk.points.begin());
    path_pub[6]->publish(mk);
}

void Ros_Utility_t::set_origin_pos(const Eigen::Vector3d &origin_pos_){
    if(!ready) return;
    origin_pos = origin_pos_;
}