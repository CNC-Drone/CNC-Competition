#ifndef __PARAM_HPP_
#define __PARAM_HPP_

#include <Eigen/Eigen>

class Param_t{
public:
    bool readParam(){
        std::string config_path_ = std::string(getenv("HOME"))+"/demon_packages/demon_drone_ws/src/demon_core/config/demon.yaml";
        // 坐标转换参数初始化
        std::cout << "\033[32m" << "[Demon_Core_t] 开始读取配置文件:" << config_path_ << "\033[0m" << std::endl;
        FILE *fh = fopen((config_path_).c_str(),"r");
        if(fh == NULL){
            std::cout << "\033[31m" << "[Demon_Core_t] failed to open file: " << config_path_ << "\033[0m" << std::endl;
            return false;          
        }
        fclose(fh);

        cv::FileStorage fsSettings((config_path_).c_str(), cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
            std::cout << "\033[31m" << "[Demon_Core_t] failed to open file: " << config_path_ << "\033[0m" << std::endl;
            return false;
        }
        cv::Mat T_ib_mat, T_ic_mat;
        
        fsSettings["config_folder"]             >> config_folder;
        fsSettings["fc_serialPath"]             >> fc_serialPath;
        fsSettings["udp_address"]               >> udp_address;
        fsSettings["if_record_video"]           >> if_record_video;
        fsSettings["if_use_ros"]                >> if_use_ros;

        fsSettings["if_launch_mavlink"]         >> if_launch_mavlink; 
        fsSettings["if_launch_vins"]            >> if_launch_vins; 
        fsSettings["if_launch_planner"]         >> if_launch_planner;
        fsSettings["if_launch_people_detect"]   >> if_launch_people_detect;
        fsSettings["if_show_info"]              >> if_show_info;
        fsSettings["if_show_people_detect"]     >> if_show_people_detect;
        fsSettings["if_use_gps"]                >> if_use_gps; 
        fsSettings.release();

        std::cout << "\033[32m" << "[Demon_Core_t] 开始读取配置文件:" << config_folder + "/stereo_config.yaml" << "\033[0m" << std::endl;
        fh = fopen((config_folder + "/stereo_config.yaml").c_str(),"r");
        if(fh == NULL){
            std::cout << "\033[31m" << "[Demon_Core_t] failed to open file: " << config_folder + "/stereo_config.yaml" << "\033[0m" << std::endl;
            return false;          
        }
        fclose(fh);
        cv::FileStorage fsSettings1((config_folder + "/stereo_config.yaml").c_str(), cv::FileStorage::READ);
        fsSettings1["imu_T_body"] >> T_ib_mat;
        fsSettings1["imu_T_cam0"] >> T_ic_mat;
        fsSettings1.release();

        T_ib = Eigen::Isometry3d::Identity();
        T_ic = Eigen::Isometry3d::Identity();
        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                T_ib(i,j) = T_ib_mat.at<double>(i,j);
                T_ic(i,j) = T_ic_mat.at<double>(i,j);
            }
        }
        
        std::cout << "\033[33m" << "*************** [Demon_Core_t] 完成配置文件读取！ ***************" << "\033[0m" << std::endl << std::endl;
        return true;      
    }

    std::string config_folder;
    std::string fc_serialPath;
    std::string udp_address;
    
    int if_launch_mavlink;

    int if_show_people_detect;
    int if_use_ros;
    int if_launch_vins;
    int if_show_info;

    int if_launch_planner;
    int if_launch_people_detect;
    int if_use_gps;
    int if_record_video;
    Eigen::Isometry3d T_ib,T_ic;        // 变换矩阵，坐标转换使用
};

#endif
