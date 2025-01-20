#ifndef __ROS_UTILITY_HPP
#define __ROS_UTILITY_HPP

#include <ros/ros.h>                            // ros头文件
#include <cv_bridge/cv_bridge.h>                // ros图像类
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>                    // imu
#include <nav_msgs/Odometry.h>                  // 里程计
#include <sensor_msgs/PointCloud2.h>            // 点云
#include <geometry_msgs/PoseStamped.h>          // 位置
#include <pcl_conversions/pcl_conversions.h>    // PCL点云转ros点云 sudo apt install ros-melodic-pcl*
#include <tf/transform_broadcaster.h>           // 坐标系tf发布
#include <visualization_msgs/Marker.h>          // 可视化模型
#include <cv_bridge/cv_bridge.h>                // 图像转换
#include <opencv2/opencv.hpp>

#define USE_ROS1

class Ros_Utility_t{
public: 
    ~Ros_Utility_t(void){ready = false;}
    void spin(int argc, char** argv, bool if_pub_stereo=false, bool if_pub_demon=false, bool if_pub_planner=false);
    void pub_Image(double img_t, cv::Mat &lFrame, cv::Mat &rFrame);
    void pub_depth(double img_t, cv::Mat &depth_img);
    void pub_imu(double imu_t, Eigen::Vector3d &acc, Eigen::Vector3d &gyr);
    void pub_gps(double gps_t, double lat, double lon, double alt, int satellites_visible){}
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
    image_transport::Publisher  lImage_pub; // 左图像发布
    image_transport::Publisher  rImage_pub; // 右图像发布
    image_transport::Publisher  lImage_rect_pub; // 左校正图像发布
    image_transport::Publisher  rImage_rect_pub; // 右校正图像发布    
    image_transport::Publisher  depth_pub;  // 深度图像发布
    ros::Publisher  lCam_info_pub;  // 右相机消息发布
    ros::Publisher  rCam_info_pub;  // 右相机消息发布
    ros::Publisher  imu_pub;        // IMU消息发布
    ros::Publisher  odom_pub;       // 里程计消息发布
    ros::Publisher  camPos_pub;     // 相机位置消息发布
    ros::Publisher  pointCloud_pub; // 点云消息发布
    ros::Publisher  vis_drone_pub;  // [发布者] 无人机模型
    ros::Publisher  path_pub[7];    // 点云消息发布
    // 数据存储
    sensor_msgs::CameraInfo  lCam_info;
    sensor_msgs::CameraInfo  rCam_info;
    sensor_msgs::PointCloud2 pointCloud_msg;

    Eigen::Vector3d origin_pos;
};


#endif
