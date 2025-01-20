#ifndef __DEMON_CORE_HPP
#define __DEMON_CORE_HPP

#include <chrono>
#include <queue>
#include <mutex>
#include <map>

#include <stereo_match.hpp>
#include <estimator/estimator.h>
#include <estimator/parameters.h>
#include <global_fusion/globalOpt.h>
#include <mavlink_manager.hpp>
#include <plan_manage/adaptive_replan_fsm.h>
#include <people_detect/people_detect.hpp>
#include <controller.h>

#include "param.hpp"
#include "ros_utility.hpp"

using namespace adaptive_planner;

struct Pos_Msg_t{
    double t;
    Eigen::Vector3d pos;
    Eigen::Quaterniond quad;
    Eigen::Vector3d euler;
};

class Demon_Core_t{
public:
    Demon_Core_t();
    ~Demon_Core_t(void);
    void spin(int argc, char** argv);
private:
    // 类
    Param_t             parm;               // 参数类
    Stereo_Match_t      stereoMatch;        // 双目类
    Estimator           estimator;          // vins类
    GlobalOptimization  globalEstimator;    // 全局定位（融合GPS信息）
    Mavlink_Manager_t   mav_msg;            // mavlink类
    AdaptiveReplanFsm   adaptive_replan;    // 路径规划类
    People_Detect_t     people_detect;      // 目标识别类
    Ros_Utility_t       ros;                // ros类
    LinearControl       controller;         // 线性控制器

    // 数据存储
    queue<std::vector<double> > gps_que;    // GPS数据队列
    std::mutex gps_que_mutex;               // GPS数据锁
    Eigen::Vector3d mag;                    // 磁力计数据

    queue<Pos_Msg_t> camPos_que;            // 相机位置队列
    std::mutex camPos_que_mutex;            // 相机位置队列锁

    queue<std::vector<DetectBox> >  people_box_que; // 行人框队列
    queue<double> people_box_t_que;                 // 行人框时间戳队列
    std::mutex people_box_que_mutex;                // 行人框锁
    unordered_map<int, Eigen::Vector3d> peoples_pos;// 行人世界坐标系位置<id, pos>

    int lock_people_id = 1;                         // 锁定的行人id

    struct{
        uint8_t SWA, SWB, SWC, SWD;
        uint8_t last_mode = -1;
        uint8_t mode = -1;
    }rc;
    enum{NONE_MODE = 0, MANUAL_MODE, OFFBOARD_MANUAL_MODE, OFFBOARD_NAVI_MODE, OFFBOARD_TEST_MODE};
    // 状态
    bool isOpen = false;
    bool if_mavlink_ready   = false;
    bool if_stereo_ready    = false;
    bool if_vins_ready      = false;
    bool if_planner_ready   = false;
    bool if_so3_ready       = false;
    bool if_people_detect_ready = false;

    void get_img_thread(void);
    void get_img_rect_thread(void);
    void get_imu_thread(void);
    void get_gps_thread(void);
    void get_rc_thread(void);
    void get_depth_Pose_thread(void);
    void get_odom_thread(void);
    void get_navigation_thread(void);
    void visualization_thread(void);
    void so3_controll_thread(void);

    std::string mode_to_string(uint8_t _mode){
        std::string res;
        if(_mode == NONE_MODE)                  res = "NONE_MODE";
        else if(_mode == MANUAL_MODE)           res = "MANUAL_MODE";
        else if(_mode == OFFBOARD_MANUAL_MODE)  res = "OFFBOARD_MANUAL_MODE";
        else if(_mode == OFFBOARD_NAVI_MODE)    res = "OFFBOARD_NAVI_MODE";
        else if(_mode == OFFBOARD_TEST_MODE)    res = "OFFBOARD_TEST_MODE";
        else res = "ERRO";
        return res;
    }    

    double getSteadySec(void){
        struct timespec ts {};
        (void)clock_gettime(CLOCK_MONOTONIC, &ts);
        double seconds = double(ts.tv_sec) + double(ts.tv_nsec)*1e-9;
        return seconds;
    }

    // ZYX
    Eigen::Vector3d get_euler_from_matrix(const Eigen::Matrix<double, 3, 3> &mat){
        Eigen::Vector3d euler_eigen;
        euler_eigen(0) = std::atan2(mat(2, 1), mat(2, 2));  // roll 
        euler_eigen(1) = std::atan2(-mat(2, 0), std::sqrt(mat(2, 1) * mat(2, 1) + mat(2, 2) * mat(2, 2)));
        euler_eigen(2) = std::atan2(mat(1, 0), mat(0, 0));  // yaw

        // // yaw (z-axis rotation)
        // double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
        // double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
        // euler_eigen[2] = std::atan2(siny_cosp, cosy_cosp);

        // // pitch (y-axis rotation)
        // double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
        // if (std::abs(sinp) >= 1)
        //     euler_eigen[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        // else
        //     euler_eigen[1] = std::asin(sinp);

        // // roll (x-axis rotation)
        // double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
        // double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
        // euler_eigen[1] = std::atan2(sinr_cosp, cosr_cosp);

        return euler_eigen;
    }
};

#endif
