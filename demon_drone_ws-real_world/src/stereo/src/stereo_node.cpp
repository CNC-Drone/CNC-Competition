#include "stereo.h"
#include <math.h>

Stereo_t::Stereo_t(std::string config_path):nh("~")
{   
    // 发布者
    image_transport::ImageTransport it(nh);  //  类似ROS句柄
    lImage_pub      = it.advertise("/stereo/left/image_raw", 1);
    rImage_pub      = it.advertise("/stereo/right/image_raw", 1);
    lImage_rect_pub = it.advertise("/stereo/left/image_rect", 1);
    rImage_rect_pub = it.advertise("/stereo/right/image_rect", 1);
    depth_pub       = it.advertise("/stereo/depth", 1);
    // lCam_info_pub   = nh.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 1);
    // rCam_info_pub   = nh.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 1);

    // 双目初始化
    if(stereoMatch.init(config_path + "/stereo_config.yaml")){
        std::cout << "\033[33m" << "*************** [Demon_Core_t] stereoMatch初始化完成 ***************" << "\033[0m" << std::endl << std::endl; 
        if_stereo_ready = true;
        get_img_timer       = nh.createTimer(ros::Duration(0.001), &Stereo_t::getImgRawCallback, this);
        get_img_rect_timer  = nh.createTimer(ros::Duration(0.001), &Stereo_t::getImgRectCallback, this);
        get_depth_timer     = nh.createTimer(ros::Duration(0.001), &Stereo_t::getDepthCallback, this);
    }
    else
        std::cout << "\033[31m" << "*************** [Demon_Core_t] stereoMatch初始化失败 ***************" << "\033[0m" << std::endl << std::endl; 
}

void Stereo_t::getImgRawCallback(const ros::TimerEvent&)
{
    static double last_img_t = 0;
    double img_t = stereoMatch.lFrame.seconds;

    if(img_t - last_img_t > 0.01){  // 图像更新了
        last_img_t = img_t;
        img_t = ros::Time::now().toSec() + (img_t - getSteadySec());  

        if(lImage_pub.getNumSubscribers() == 0 && rImage_pub.getNumSubscribers() == 0) return;
        static sensor_msgs::Image left_img, right_img;
        static std_msgs::Header header;
        header.frame_id = "stereo";
        header.stamp.sec = int(img_t);
        header.stamp.nsec = (img_t - header.stamp.sec) * 1e9;
        cv_bridge::CvImage(header, "bgr8", stereoMatch.lFrame.data).toImageMsg(left_img);
        cv_bridge::CvImage(header, "bgr8", stereoMatch.rFrame.data).toImageMsg(right_img);

        lImage_pub.publish(left_img);
        rImage_pub.publish(right_img);
    }
}

void Stereo_t::getImgRectCallback(const ros::TimerEvent&)
{
    static double last_img_t = 0;
    double img_t = stereoMatch.lFrame_rec.seconds;

    if(img_t - last_img_t > 0.01){  // 图像更新了
        last_img_t = img_t;
        img_t = ros::Time::now().toSec() - getSteadySec() + img_t;

        if(lImage_rect_pub.getNumSubscribers() == 0 && rImage_rect_pub.getNumSubscribers() == 0) return;
        static sensor_msgs::Image left_img, right_img;
        static std_msgs::Header header;

        header.frame_id = "stereo";
        header.stamp.sec = int(img_t);
        header.stamp.nsec = (img_t - header.stamp.sec) * 1e9;
        cv_bridge::CvImage(header, "bgr8", stereoMatch.lFrame_rec.data).toImageMsg(left_img);
        cv_bridge::CvImage(header, "bgr8", stereoMatch.rFrame_rec.data).toImageMsg(right_img);

        lImage_rect_pub.publish(left_img);
        rImage_rect_pub.publish(right_img);
    }
}

void Stereo_t::getDepthCallback(const ros::TimerEvent&)
{
    static double last_img_t = 0;
    double img_t = stereoMatch.depth_img.seconds;

    if(img_t - last_img_t > 0.01){  // 图像更新了
        last_img_t = img_t;
        img_t = ros::Time::now().toSec() - getSteadySec() + img_t;

        if(depth_pub.getNumSubscribers() == 0) return;
        static sensor_msgs::Image depth_msg;
        static std_msgs::Header header;

        header.frame_id = "stereo";
        header.stamp.sec = int(img_t);
        header.stamp.nsec = (img_t - header.stamp.sec) * 1e9;
        cv_bridge::CvImage(header, "mono16", stereoMatch.depth_img.data).toImageMsg(depth_msg);
        depth_pub.publish(depth_msg); 
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereo_node");   // ros初始化，定义节点名为
    ros::MultiThreadedSpinner spinner(0);


    if(argc != 2)
    {
        std::cout << "\033[31m" << "*************** [Erro] 请输入配置文件地址 ***************" << "\033[0m" << std::endl << std::endl; 
        return 1;
    }
    std::string config_file_path = argv[1];

    Stereo_t stereo(config_file_path);
    ROS_INFO("[INFO] stereo_node launch.\n");  
    spinner.spin();
    ROS_INFO("[INFO] stereo_node closed.\n");    
}
