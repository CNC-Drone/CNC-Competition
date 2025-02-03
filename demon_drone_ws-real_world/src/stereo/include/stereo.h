#ifndef __STEREO_H
#define __STEREO_H

#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>  // 将ROS下的sensor_msgs/Image消息类型转化为cv::Mat数据类型
#include <sensor_msgs/image_encodings.h> // ROS下对图像进行处理
#include <image_transport/image_transport.h> // 用来发布和订阅图像信息
#include <tf/tf.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <stereo_match.hpp>

class Stereo_t{
public:
    Stereo_t(std::string config_path);
    ~Stereo_t(){
        std::cout << "程序中止" << std::endl;
    }

    ros::NodeHandle nh;             // ros句柄
    Stereo_Match_t  stereoMatch;    // 双目类

    bool if_stereo_ready    = false;
// private:
    // 发布者
    image_transport::Publisher  lImage_pub;         // 左图像发布
    image_transport::Publisher  rImage_pub;         // 右图像发布
    image_transport::Publisher  lImage_rect_pub;    // 左校正图像发布
    image_transport::Publisher  rImage_rect_pub;    // 右校正图像发布    
    image_transport::Publisher  depth_pub;          // 深度图像发布
    // ros::Publisher  lCam_info_pub;  // 右相机消息发布
    // ros::Publisher  rCam_info_pub;  // 右相机消息发布


    // 定时器
    ros::Timer get_img_timer;
    ros::Timer get_depth_timer;
    ros::Timer get_img_rect_timer;  

    // 回调函数
    void getImgRawCallback(const ros::TimerEvent&);
    void getImgRectCallback(const ros::TimerEvent&);
    void getDepthCallback(const ros::TimerEvent&);

    double getSteadySec(void){
        struct timespec ts {};
        (void)clock_gettime(CLOCK_MONOTONIC, &ts);
        double seconds = double(ts.tv_sec) + double(ts.tv_nsec)*1e-9;
        return seconds;
    }
};

#endif
