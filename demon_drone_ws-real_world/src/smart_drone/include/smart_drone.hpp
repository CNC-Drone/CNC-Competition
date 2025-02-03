#ifndef __SMART_DRONE_HPP
#define __SMART_DRONE_HPP

#include <iostream>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>  // 将ROS下的sensor_msgs/Image消息类型转化为cv::Mat数据类型
#include <sensor_msgs/image_encodings.h> // ROS下对图像进行处理
#include <image_transport/image_transport.h> // 用来发布和订阅图像信息
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

#include <opencv2/opencv.hpp>
#include <people_detect/people_detect.hpp>
#include <mutex>

enum
{
    NONE=0,     // 初始化
    MANUAL,     // 手动飞行模式（扫描模式）
    TRACK,      // 跟踪模式
    ATTACK      // 攻击模式
};

class Smart_Drone_t
{
public:
    Smart_Drone_t(void);
    ~Smart_Drone_t(void)
    {
        isReady = false;
    }
private:
    People_Detect_t people_detect;  // 目标识别类
    ros::NodeHandle     nh;         // ros句柄
    cv::VideoWriter videoWriter;    // 视频记录

    // 订阅
    ros::Subscriber             rc_sub;             // 遥控器值（10Hz）
    ros::Subscriber             mavros_imu_sub;     // [订阅者] 飞控imu数据

    image_transport::Subscriber left_rect_img_sub;  // 左校正图像订阅
    // 发布
    ros::Publisher  setpoint_pub;                   // [发布者] px4控制点（位置、速度、加速度）

    sensor_msgs::Imu   mavros_imu;              // [订阅消息存储] 飞控imu数据

    // 图像相关
    sensor_msgs::Image lRect_img;
    cv_bridge::CvImageConstPtr ptr;
    std::mutex  lRect_img_mutex; 
    int image_width     = 0;
    int image_height    = 0;

    bool isReady = false;
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

    struct
    {
        double t;
        DetectBox chosen_box;   // 目标物体框
    }target_info;
    
    struct
    {
        // 目标识别相关参数
        std::string modelPath;
        int if_show_dect;
    }parm;
    int mode = NONE;

    // 函数
    double getNearestBox(std::vector<DetectBox> &boxes, int x, int y, DetectBox* nearestBox);
    double limit(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    // callback
    void lrect_img_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        lRect_img_mutex.unlock();        
    }
    void rc_callback(const mavros_msgs::RCIn::ConstPtr& msg);
    void mavros_imu_callback(const sensor_msgs::Imu::ConstPtr& msg){            // mavros imu数据回调
        mavros_imu = *msg;
    }
    // 线程函数
    void Object_Detect_Thread(void);
    void Drone_Control_Thread(void);
};

#endif
