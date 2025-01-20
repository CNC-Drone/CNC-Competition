#ifndef __TRANSFER_HPP__
#define __TRANSFER_HPP__

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/RCIn.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <mutex>
#include <thread>
#include <termio.h>
#include <math.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <geodesy/utm.h>    // sudo apt-get install ros-melodic-geodesy
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPointStamped.h>

#define SIGN(x)  ((x)>0?1:((x)<0?-1:0))

/* 宏常量 */
#define STARTING	0//< 启动时参数
#define RUNNING		1//< 运行时参数
#define ENDING		2//< 结束时参数

#define COUT(X) std::cout <<std::setiosflags(std::ios::fixed)<< X << "\033[3m" << "\r" << std::flush << std::endl
#define COUTR(X) std::cout <<std::setiosflags(std::ios::fixed)<< "\033[1;31m" << X << "\033[0m" << "\r" << std::flush << std::endl  // red
#define COUTG(X) std::cout <<std::setiosflags(std::ios::fixed)<< "\033[1;32m" << X << "\033[0m" << "\r" << std::flush << std::endl  // green
#define COUTY(X) std::cout <<std::setiosflags(std::ios::fixed)<< "\033[1;33m" << X << "\033[0m" << "\r" << std::flush << std::endl  // yellow
#define COUTB(X) std::cout <<std::setiosflags(std::ios::fixed)<< "\033[1;34m" << X << "\033[0m" << "\r" << std::flush << std::endl  // blue
#define COUTP(X) std::cout <<std::setiosflags(std::ios::fixed)<< "\033[1;35m" << X << "\033[0m" << "\r" << std::flush << std::endl  // purple

float Limit(float data,float max,float min);

enum{
    Safe_Mode = 0,
    Stabilized_Mode,
    Position_Mode,
    Land_Mode,
    RC_ctr_Mode,
    Keyboard_ctr_mode,
    Traj_ctr_Mode,
    Turn_Back_Mode,
};


class Transfer{
public:
    struct{ 
        float camer_offset_x = 0;  // 相对于无人机中心
        float camer_offset_y = 0;
        float camer_offset_z = 0;
        float camer_theta;
        int ctrFreq, ifShowInfo, useSelfApp;
        float pid_xy_p, pid_z_p, pid_yaw_p, max_xy_vel, max_z_vel;
        std::string target_pose_w_sub_topic;
        cv::Mat T_ib_mat, T_ic_mat;
        Eigen::Isometry3d T_ib,T_ic;
        
        float RC_Z_RATIO;
        float RC_XY_RATIO;
        float RC_YAW_RATIO;
    }parm;
    float vision_odom_freq = 0;
    float mavros_odom_freq = 0;
    uint16_t mode = Safe_Mode, last_mode = Safe_Mode;
    FILE *fp_path, *setPoint_fp, *gps_fp, *human_gps_fp, *car_gps_fp;
    ros::NodeHandle nh;
    bool gnss_init = false;
    float mag_yaw;
    nav_msgs::Odometry GPS_odom;
    Eigen::Vector3d GPS_init_pos;
public:
    Transfer(std::string config_path);
    ~Transfer(){
        fclose(fp_path);
        fclose(setPoint_fp);
        fclose(gps_fp);
        fclose(human_gps_fp);
        fclose(car_gps_fp);
    }
private:
    ros::Subscriber vision_odom_sub;     // [订阅者] 视觉惯性里程计
    ros::Subscriber trajectory_pose_sub; // [订阅者] 路径规划轨迹点
    ros::Subscriber rc_sub;              // [订阅者] 遥控器值（10Hz）
    ros::Subscriber mavros_imu_sub;      // [订阅者] 飞控imu数据
    ros::Subscriber mavros_state_sub;    // [订阅者] 飞控状态数据
    ros::Subscriber mavros_odom_sub;     // [订阅者] 飞控里程计
    ros::Subscriber mavros_gps_sub;      // [订阅者] 飞控GPS
    ros::Subscriber car_gps_sub;        // [订阅者] car_GPS
    ros::Subscriber mavros_mag_sub;     // [订阅者] 飞控磁力计
    ros::Subscriber human_pos_sub;       // [订阅者] 订阅追踪目标的位置点
    ros::Subscriber track_pos_sub;       // [订阅者] 订阅最终追踪目标的位置点
    ros::Subscriber keyboard_pos_sub;    // [订阅者] 订阅键盘输入的位置信息

    ros::Publisher  setpoint_pub;        // [发布者] px4控制点（位置、速度、加速度）
    ros::Publisher  camera_pose_pub;     // [发布者] 发布相机位置（主要在路径规划中地图构建会用到）
    ros::Publisher  drone_odom_pub;      // [发布者] 发布无人机里程计（其实还是相机里程计，只是转角不一样）
    ros::Publisher  drone_pose_pub;      // [发布者] 发布无人机位置
    ros::Publisher  drone_vel_pub;       // [发布者] 发布无人机速度
    ros::Publisher  vins_restart_pub;    // [发布者] 视觉惯性里程计重启
    ros::Publisher  waypoint_pub;        // [发布者] 设置路径规划目标点
    ros::Publisher  vis_drone_pub;       // [发布者] 无人机模型
    ros::Publisher  human_pos_init_pub;  // [发布者] 发布初始追踪位置
    ros::Publisher  human_pos_gps_pub;   // [发布者] 发布行人GPS位置    

    struct{
        uint8_t SWA=0;
        uint8_t SWB=0;
        uint8_t SWC=0;
        uint8_t SWD=0;
        int16_t Throttle=0; // 通道3，向上为正（-500  -  500）
        int16_t Roll=0;     // 通道1，向右为正（-500  -  500）
        int16_t Pitch=0;    // 通道2，向上为负（-500  -  500）
        int16_t Yaw=0;      // 通道4，向右为正（-500  -  500）
    }rc;  // [订阅消息存储] 遥控器值结构体
    sensor_msgs::Imu   mavros_imu;              // [订阅消息存储] 飞控imu数据
    mavros_msgs::State mavros_state;            // [订阅消息存储] 飞控状态数据 .mode .armed .connected
    nav_msgs::Odometry mavros_odom;             // [订阅消息存储] 飞控里程计
    geographic_msgs::GeoPointStamped gps_msg;   // [订阅消息存储] 飞控GPS
    int satellites_num;                         // [订阅消息存储] 飞控GPS卫星数量

    nav_msgs::Odometry vins_odom;                       // [订阅消息存储] 视觉里程计
    quadrotor_msgs::PositionCommand trajectory_pose;    // [订阅消息存储] 路径规划轨迹点    
    geometry_msgs::PoseStamped human_pos;       // [订阅消息存储] 追踪目标的位置点
    geometry_msgs::PoseStamped track_pos;       // [订阅消息存储] 最终追踪目标的位置点
    mavros_msgs::PositionTarget setpoint;       // [发送消息存储] px4控制点（位置、速度、加速度）
    geometry_msgs::PoseStamped camera_pose;     // [发送消息存储] 相机位置
    nav_msgs::Odometry drone_odom;              // [发送消息存储] 无人机里程计
    sensor_msgs::NavSatFix human_pos_gps;       // [发送消息存储] 行人GPS位置

    Eigen::Isometry3d T_wb, T_wc, T_gb;
    // 同步和互斥变量
    std::mutex vins_ready;
    bool trajectory_pose_ready = false;
    bool target_pos_ready = false;
    bool human_pose_ready = false;         
private:
    void readParameters(std::string config_path);
    void vision_odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
        vins_odom = *msg;
        vins_ready.unlock();
    }
    void trajectory_pose_callback(const quadrotor_msgs::PositionCommand::ConstPtr& msg){      // 路径规划轨迹回调
        trajectory_pose = *msg;
        Eigen::Vector3d traj_pos(trajectory_pose.position.x, trajectory_pose.position.y, trajectory_pose.position.z);
        Eigen::Vector3d drone_pos(drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z);
        if((traj_pos - drone_pos).norm() < 0.6)  trajectory_pose_ready = true;
    }
    void rc_callback(const mavros_msgs::RCIn::ConstPtr& msg);
    void mavros_imu_callback(const sensor_msgs::Imu::ConstPtr& msg){            // mavros imu数据回调
        mavros_imu = *msg;
    }
    void mavros_mag_callback(const sensor_msgs::MagneticField::ConstPtr& msg){  // mavros mag数据回调
        geometry_msgs::Vector3 euler;
        tf::Matrix3x3 mat;
        mat = tf::Matrix3x3(tf::Quaternion(mavros_imu.orientation.x, mavros_imu.orientation.y, mavros_imu.orientation.z, mavros_imu.orientation.w));
        mat.getRPY(euler.x, euler.y, euler.z);
        // mag_yaw = atan2(-msg->magnetic_field.y*cos(euler.x)+msg->magnetic_field.z*sin(euler.x),
        //     (msg->magnetic_field.x*cos(euler.y) + msg->magnetic_field.y*sin(euler.y)*sin(euler.x) + msg->magnetic_field.z*sin(euler.y)*cos(euler.x)));
        mag_yaw = -atan2(-msg->magnetic_field.x*cos(euler.y)+msg->magnetic_field.z*sin(euler.y),
            (msg->magnetic_field.y*cos(euler.x) + msg->magnetic_field.x*sin(euler.x)*sin(euler.y) + msg->magnetic_field.z*sin(euler.x)*cos(euler.y)));
    }
    void mavros_state_callback(const mavros_msgs::State::ConstPtr& msg){        // mavros state数据回调
        mavros_state = *msg;
    } 
    void mavros_odom_callback(const nav_msgs::Odometry::ConstPtr& msg){         // mavros odom数据回调       
        mavros_odom = *msg;
    }
    void mavros_gps_callback(const mavros_msgs::GPSRAW::ConstPtr& msg);
    void car_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
        geographic_msgs::GeoPointStamped car_gps_msg;
        car_gps_msg.header = msg->header;
        car_gps_msg.position.latitude = msg->latitude;
        car_gps_msg.position.longitude = msg->longitude;
        car_gps_msg.position.altitude = msg->altitude;      
        geodesy::UTMPoint utm;
        geodesy::fromMsg(car_gps_msg.position, utm);
        if(gnss_init){
            fprintf(car_gps_fp, "%lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf\n", msg->header.stamp.toSec(), utm.easting - GPS_init_pos[0], utm.northing - GPS_init_pos[1], utm.altitude - GPS_init_pos[2],
            1,0,0,0);  
        }
    }
    void human_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        human_pos = *msg;
        human_pose_ready = true;
    }
    void track_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        track_pos = *msg;
        target_pos_ready = true;
    }    
    void keyboard_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void setpoint_pub_fun(bool if_record = true);
    void getCameraPos();
    void vis_drone_model();

    std::string mode_to_string(uint16_t _mode);
    void showInfo(void);
    void Remote_Handle(void);
    void tf_process(void);
    Eigen::Vector3d pose_c_2_w(Eigen::Vector3d pos_c);
};


#endif
