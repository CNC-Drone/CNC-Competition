#ifndef __OBJECT_IDENTIFY_HPP
#define __OBJECT_IDENTIFY_HPP

#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>  // 将ROS下的sensor_msgs/Image消息类型转化为cv::Mat数据类型
#include <sensor_msgs/image_encodings.h> // ROS下对图像进行处理
#include <image_transport/image_transport.h> // 用来发布和订阅图像信息

#include <people_detect/people_detect.hpp>
#include <mutex>
#include <object_detection_msgs/BoundingBox.h>
#include <object_detection_msgs/BoundingBoxes.h>

class Object_Identify_t
{
public:
    Object_Identify_t(std::string config_path);
    ~Object_Identify_t(){isReady = false;}

    ros::NodeHandle     nh;             // ros句柄
    People_Detect_t     people_detect;  // 目标识别类
private:
    // 订阅
    image_transport::Subscriber left_rect_img_sub;
    ros::Publisher bboxes_pub;

    sensor_msgs::Image lRect_img;
    cv_bridge::CvImageConstPtr ptr;
    bool isReady = false;
    std::mutex  lRect_img_mutex; 

    // 回调函数
    void lrect_img_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        // if (msg->encoding == "8UC1")
        // {
        //     lRect_img.header = msg->header;
        //     lRect_img.height = msg->height;
        //     lRect_img.width = msg->width;
        //     lRect_img.is_bigendian = msg->is_bigendian;
        //     lRect_img.step = msg->step;
        //     lRect_img.data = msg->data;
        //     lRect_img.encoding = "mono8";
        //     ptr = cv_bridge::toCvCopy(lRect_img, sensor_msgs::image_encodings::MONO8);
        // }
        // else
        ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        lRect_img_mutex.unlock();
    }

    void get_img_rect_thread(void);
};


#endif
