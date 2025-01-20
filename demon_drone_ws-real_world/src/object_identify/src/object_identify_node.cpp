#include "object_identify.hpp"


Object_Identify_t::Object_Identify_t(std::string config_path):nh("~")
{
    image_transport::ImageTransport it(nh);  //  类似ROS句柄
    left_rect_img_sub = it.subscribe("/stereo/left/image_rect", 1, &Object_Identify_t::lrect_img_callback, this);
    bboxes_pub = nh.advertise<object_detection_msgs::BoundingBoxes>("/target/odom", 1);

    lRect_img_mutex.lock();
    // std:cout << "模型地址：" + config_path + "/models" << std::endl;
    if(people_detect.init(config_path + "/models"))
    {
        isReady = true;
        get_img_rect_thread();
        std::cout << "\033[33m" << "*************** [Object_Identify_t] people_detect初始化完成 ***************" << "\033[0m" << std::endl << std::endl; 
    }
    
}


void Object_Identify_t::get_img_rect_thread(void){
    std::thread ThRecv	= std::thread{
        [&](){
            std::cout << "\033[32m" << "[Object_Identify_t] 获取校正图像线程开始......" << "\033[0m" << std::endl;
            while (ros::ok() && isReady){
                lRect_img_mutex.lock();
                cv::Mat lFrame_rec = ptr->image.clone();
                // cv::imshow("left", lFrame_rec);
                people_detect.process(lFrame_rec, true);    // true的时候显示图像，false不显示

                object_detection_msgs::BoundingBoxes bboxes_msg;
                for(int i = 0; i < people_detect.people_boxs.size(); ++i){
                    // if(people_detect.people_boxs[i].classID != 0) continue;
                    // if(people_detect.people_boxs[i].confidence < 0.35) continue;
                    bboxes_msg.header.stamp = ros::Time::now();
                    bboxes_msg.image_header.stamp = ros::Time::now();
                    object_detection_msgs::BoundingBox bbox_msg;
                    bbox_msg.xmin = people_detect.people_boxs[i].x1;
                    bbox_msg.ymin =  people_detect.people_boxs[i].y1;
                    bbox_msg.xmax = people_detect.people_boxs[i].x2;
                    bbox_msg.ymax = people_detect.people_boxs[i].y2;
                    bboxes_msg.bounding_boxes.emplace_back(bbox_msg);
                }
                bboxes_pub.publish(bboxes_msg);
            }
            std::cout << "\033[34m" << "[Object_Identify_t] 结束获取校正图像线程！" << "\033[0m" << std::endl;
        }
    };
    ThRecv.detach();
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_identify_node");   // ros初始化，定义节点名为
    ros::MultiThreadedSpinner spinner(0);

    if(argc != 2)
    {
        std::cout << "\033[31m" << "*************** [Erro] 请输入配置文件地址 ***************" << "\033[0m" << std::endl << std::endl; 
        return 1;
    }
    std::string config_file_path = argv[1];

    Object_Identify_t object_identify(config_file_path);
    ROS_INFO("[INFO] object_identify_node launch.\n");  
    spinner.spin();
    ROS_INFO("[INFO] object_identify_node closed.\n");    
}

