#ifndef __RECORD_LOG_HPP
#define __RECORD_LOG_HPP

#include <eigen3/Eigen/Eigen>
#include <fstream>

cv::VideoWriter videoWriter;

/************************************** 日志输出相关 *********************************************/
void record_vb_odom(double t, const Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Quaterniond &quad){
    static bool if_init = false;
    if(!if_init){
        if_init = true;
        std::ofstream pose_fd((std::string(getenv("HOME"))+"/demon_drone_ws/output/vio_pos.csv").c_str(), std::ios::out);
        pose_fd.close();
        std::ofstream vel_fd((std::string(getenv("HOME"))+"/demon_drone_ws/output/drone_vel.csv").c_str(), std::ios::out);
        vel_fd.close();                  
    }

    std::ofstream pose_fd((std::string(getenv("HOME"))+"/demon_drone_ws/output/vio_pos.csv").c_str(), std::ios::app);  // tum格式
    pose_fd.setf(std::ios::fixed, std::ios::floatfield);
    pose_fd.precision(2);
    pose_fd << t << " ";
    pose_fd.precision(2);
    pose_fd << pos[0] << " " << pos[1] << " " << pos[2] << " " << quad.x() << " " << quad.y() << " " << quad.z() << " " << quad.w() << std::endl;
    pose_fd.close();

    std::ofstream vel_fd((std::string(getenv("HOME"))+"/demon_drone_ws/output/drone_vel.csv").c_str(), std::ios::app);
    vel_fd.setf(std::ios::fixed, std::ios::floatfield);
    vel_fd.precision(2);
    vel_fd << t << " ";
    vel_fd.precision(2);
    vel_fd << vel[0] << " " << vel[1] << " " << vel[2] << std::endl;
    vel_fd.close();        
}

void record_GPS_pos(double t, const Eigen::Vector3d &pos){
    static bool if_init = false;
    if(!if_init){
        if_init = true;
        std::ofstream pose_fd((std::string(getenv("HOME"))+"/demon_drone_ws/output/gps_pos.csv").c_str(), std::ios::out);
        pose_fd.close();              
    }

    std::ofstream pose_fd((std::string(getenv("HOME"))+"/demon_drone_ws/output/gps_pos.csv").c_str(), std::ios::app);  // tum格式
    pose_fd.setf(std::ios::fixed, std::ios::floatfield);
    pose_fd.precision(2);
    pose_fd << t << " ";
    pose_fd.precision(2);
    pose_fd << pos[0] << " " << pos[1] << " " << pos[2] << " " << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
    pose_fd.close();    
}

void record_gb_pos(double t, const Eigen::Vector3d &pos, Eigen::Quaterniond &quad){
    static bool if_init = false;
    if(!if_init){
        if_init = true;
        std::ofstream pose_fd((std::string(getenv("HOME"))+"/demon_drone_ws/output/globa_pos.csv").c_str(), std::ios::out);
        pose_fd.close();              
    }

    std::ofstream pose_fd((std::string(getenv("HOME"))+"/demon_drone_ws/output/globa_pos.csv").c_str(), std::ios::app);  // tum格式
    pose_fd.setf(std::ios::fixed, std::ios::floatfield);
    pose_fd.precision(2);
    pose_fd << t << " ";
    pose_fd.precision(2);
    pose_fd << pos[0] << " " << pos[1] << " " << pos[2] << " " << quad.x() << " " << quad.y() << " " << quad.z() << " " << quad.w() << std::endl;
    pose_fd.close();    
}

void record_waypoint(double t, const Eigen::Vector3d &pos){
    static bool if_init = false;
    if(!if_init){
        if_init = true;
        std::ofstream pose_fd((std::string(getenv("HOME"))+"/demon_drone_ws/output/waypoint.csv").c_str(), std::ios::out);
        pose_fd.close();              
    }

    std::ofstream pose_fd((std::string(getenv("HOME"))+"/demon_drone_ws/output/waypoint.csv").c_str(), std::ios::app);  // tum格式
    pose_fd.setf(std::ios::fixed, std::ios::floatfield);
    pose_fd.precision(2);
    pose_fd << t << " ";
    pose_fd.precision(2);
    pose_fd << pos[0] << " " << pos[1] << " " << pos[2] << " " << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
    pose_fd.close();
}

void record_video(cv::Mat &img){
    static bool if_init = false;
    if(!if_init){
        if_init = true;
        videoWriter.open(std::string(getenv("HOME"))+"/demon_drone_ws/output/drone.avi"  // 地址
                                , cv::VideoWriter::fourcc('M', 'J', 'P', 'G')       // 格式
                                , 10                                                // 帧率
                                , img.size());                                      // 尺寸
    }
    if(videoWriter.isOpened()) videoWriter.write(img);
}

void record_close(void){
    if(videoWriter.isOpened())  videoWriter.release();
}

#endif

