#ifndef __ROS_UTILITY_HPP
#define __ROS_UTILITY_HPP

#include <rclcpp/rclcpp.hpp>                    // ros头文件
#include <sensor_msgs/msg/image.hpp>            // ros图像类
#include <sensor_msgs/msg/imu.hpp>              // imu
#include <nav_msgs/msg/odometry.hpp>            // 里程计
#include <sensor_msgs/msg/point_cloud2.hpp>     // 点云
#include <geometry_msgs/msg/pose_stamped.hpp>   // 位置
#include <pcl_conversions/pcl_conversions.h>    // sudo apt install ros-humble-pcl*
#include <visualization_msgs/msg/marker.hpp>    // 可视化模型
#include <cv_bridge/cv_bridge.h>                // 图像转换
#include <tf2_ros/transform_broadcaster.h>      // 坐标系tf发布
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <opencv2/opencv.hpp>

#define USE_ROS2

class Ros_Utility_t{
public: 
    ~Ros_Utility_t(void){ready = false;}
    void spin(int argc, char** argv, bool if_pub_stereo=false, bool if_pub_demon=false, bool if_pub_planner=false);
    void pub_Image(double img_t, cv::Mat &lFrame, cv::Mat &rFrame);
    void pub_depth(double img_t, cv::Mat &depth_img);
    void pub_imu(double imu_t, Eigen::Vector3d &acc, Eigen::Vector3d &gyr);
    void pub_gps(double gps_t, double lat, double lon, double alt, int satellites_visible);
    void pub_odom(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Quaterniond &quad);
    void pub_cam_pos(double t, Eigen::Vector3d &pos, Eigen::Quaterniond &quad);
    void pub_cam_info(void);
    void pub_image_rect(double img_t, cv::Mat &lFrame, cv::Mat &rFrame);
    void pub_point_cloud(pcl::PointCloud<pcl::PointXYZ> &cloud);
    void displayLineStrip(double t, const std::vector<Eigen::Vector3d>& list, double line_width, const Eigen::Vector4d& color, int id, int pub_id);
    void displaySphereList(double t, const std::vector<Eigen::Vector3d>& list, double resolution, const Eigen::Vector4d& color, int id, int pub_id);
    void drawAstar(double t, const std::vector<Eigen::Vector3d>& path, double line_width, const Eigen::Vector4d& color);
    void drawLowMpcTraj(double t, const std::vector<Eigen::Vector3d>& traj, double resolution, const Eigen::Vector4d& color);
    void drawLocalGoal(double t, Eigen::Vector3d local_goal, double resolution, const Eigen::Vector4d& color);
    void drawHighMpccTraj(double t, const std::vector<Eigen::Vector3d>& traj, double resolution, const Eigen::Vector4d& color);
    void drawCmd(double t, const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id, const Eigen::Vector4d& color);
    void drawHighMpccRefTraj(double t, const std::vector<Eigen::Vector3d>& traj, double resolution, const Eigen::Vector4d& color);
    void drawDroneTraj(double t, const Eigen::Vector3d &pos, const Eigen::Vector3d &vel);

    void set_origin_pos(const Eigen::Vector3d &origin_pos_);

    bool ready = false;
private:
    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           lImage_pub;     // 左图像发布
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           rImage_pub;     // 右图像发布
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           depth_pub;      // 深度图像发布
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr             imu_pub;        // IMU消息发布
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr       gps_pub;        // gps消息发布
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           odom_pub;       // 里程计消息发布
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr   camPos_pub;     // 相机位置发布
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     pointCloud_pub; // 点云消息发布
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   vis_drone_pub;  // 点云消息发布
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   path_pub[7];  // 点云消息发布

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_wb_pub;

    Eigen::Vector3d origin_pos;
};

#endif
