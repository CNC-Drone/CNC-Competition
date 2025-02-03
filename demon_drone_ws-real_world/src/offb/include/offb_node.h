#ifndef __OFFB_NODE_H__
#define  __OFFB_NODE_H__

#include <ros/ros.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/RCIn.h>
#include "/usr/include/yaml-cpp/yaml.h"
#include <iostream>
#include <vector>
#include <sensor_msgs/Imu.h>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <math.h>
#include <tf/tf.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <thread>
#include <mutex>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include "utils.h"
#include <tf/transform_datatypes.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <nodelet/nodelet.h>
#include "PX4CtrlFSM.h"
#include "controller.h"
#include <std_msgs/Bool.h>


using namespace std;
using namespace Eigen;

class Offb_control{
private:
    ros::Publisher setpoint_pub_raw, ego_goal_msg_pub, ego_cam_pose_pub, local_pose_pub, setpoint_raw_atti_pub, debug_pub, triger_pub;
    ros::Subscriber state_sub, gps_info, local_pose_info, vins_sub, rel_altitude, px4_quaternion_sub, drift_yaw_sub, quadrotor_cmd_sub, camera_depth_sub, rc_in_sub, takeoff_sub;
    ros::ServiceClient arming_client, set_mode_client, reboot_FCU_srv;

    //配置文件变量
    int use_wgs84, num_point, use_mag, so3_control_flag;
    float position_threshold, height_threshold, time_duration, yaw_set, yaw_fix_threshold, position_orientation_set, smooth;
    Eigen::Isometry3d T_i_cam;
    std::vector<double> start_point_wgs84;
    std::vector<std::vector<double>> point;  //local points
    std::vector<std::vector<double>> point_wgs84;   //wgs84 pints

    //location_cnn优化相关变量
    double last_optimized_yaw_data = -1.0;
    std_msgs::Float32 optimized_yaw;

    //初始化相关变量
    float yaw_mag_NED;     //磁航向角
    std_msgs::Float32 yaw_mag_msg;//磁航向角消息
    vector<vector<double>> point_fixed;//基于磁航向角修正目标点
    tf::Quaternion q_vins2px4; //FLU->NED
    Matrix4d V_wci_ci;
    Eigen::Isometry3d T_fi_ci, T_ci_fi;//飞控IMU坐标系下cam IMU的位姿
    int use_camIMU;

    mutex m_local_position;
    mutex m_px4_imu;
    mutex m_current_state;
    mutex m_controller;

    mavros_msgs::State current_state;//无人机状态
    nav_msgs::Odometry  local_pose;//vins转给px4的姿态
    mavros_msgs::GPSRAW gps_infomation;//GPS
    mavros_msgs::Altitude altitude_mavros;//高度
    geometry_msgs::PoseStamped local_pose_infomation;//px4融合位姿
    sensor_msgs::Imu px4_imu;
    mavros_msgs::RCIn rcin_msg;
    mavros_msgs::PositionTarget local_position_msg;//目标控制量
    mavros_msgs::AttitudeTarget attitude_msg;
    nav_msgs::Odometry vins_rect_pose;//loop_closure回调位姿
    nav_msgs::Odometry vins_pose;//vins回调位姿
    cv::Mat depth_pic_camera; //深度图
    float d_value_camera;//深度距离
    //避障相关变量
    geometry_msgs::PoseStamped ego_goal_msg;//目标点
    geometry_msgs::PoseStamped camera_pose;//相机位姿

    geometry_msgs::PoseStamped triger_msg;

    int i_interrupt = 0;
    int s = 0;

    Parameter_t param;
    Imu_Data_t imu_data;
    Odom_Data_t odom_data;
    Desired_State_t des;
    Controller_Output_t u;
    bool takeoff_triger = false;
    bool if_takeoff = false;
    ros::Time now_time, last_time;
    bool get_planning_msg = false;
    ros::Time planning_timestamp;
    bool if_land = true;
    bool land_flag;
    bool takeoff_flag;

public:
    Offb_control();
    ~Offb_control();
    void readParameters();

    ros::NodeHandle nh;//必须在ros::Time、ros::Rate之前创建节点
    ros::Time last_request;
    ros::Rate rate = 2;

    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        m_current_state.lock();
        current_state = *msg;
        m_current_state.unlock();
    }
    void gps_info_cb(const mavros_msgs::GPSRAW::ConstPtr& msg){
        gps_infomation = *msg;
    }
    void alt_cb(const mavros_msgs::Altitude::ConstPtr& msg){
        altitude_mavros = *msg;
    }
    void local_info_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        m_controller.lock();
        local_pose_infomation = *msg;
        odom_data.p(0) = local_pose_infomation.pose.position.x;
        odom_data.p(1) = local_pose_infomation.pose.position.y;
        odom_data.p(2) = local_pose_infomation.pose.position.z;
        odom_data.q.w() = local_pose_infomation.pose.orientation.w;
        odom_data.q.x() = local_pose_infomation.pose.orientation.x;
        odom_data.q.y() = local_pose_infomation.pose.orientation.y;
        odom_data.q.z() = local_pose_infomation.pose.orientation.z;
        // printf("%f    %f    %f/n",local_pose_infomation.pose.position.x, local_pose_infomation.pose.position.y, local_pose_infomation.pose.position.z);
        m_controller.unlock();
    }

    void px4_imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
        m_px4_imu.lock();
        px4_imu = *msg;
        imu_data.w(0) = px4_imu.angular_velocity.x;
        imu_data.w(1) = px4_imu.angular_velocity.y;
        imu_data.w(2) = px4_imu.angular_velocity.z;
        imu_data.a(0) = px4_imu.linear_acceleration.x;
        imu_data.a(1) = px4_imu.linear_acceleration.y;
        imu_data.a(2) = px4_imu.linear_acceleration.z;
        imu_data.q.x() = px4_imu.orientation.x;
        imu_data.q.y() = px4_imu.orientation.y;
        imu_data.q.z() = px4_imu.orientation.z;
        imu_data.q.w() = px4_imu.orientation.w;
        m_px4_imu.unlock();
    }

    void RCIn_cb(const mavros_msgs::RCIn::ConstPtr& msg){
        m_controller.lock();
        rcin_msg = *msg;
        m_controller.unlock();
    }

    void takeoff_cb(const std_msgs::Bool::ConstPtr& msg){
        takeoff_triger = msg->data;
    }

    void quadrotor_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg){
        m_controller.lock();
        if(rcin_msg.channels[6] > 1800){
            planning_timestamp = msg -> header.stamp;
            des.p.x() = msg->position.x;
            des.p.y() = msg->position.y;
            des.p.z() = msg->position.z;
            des.v.x() = msg->velocity.x;
            des.v.y() = msg->velocity.y;
            des.v.z() = msg->velocity.z;
            des.a.x() = msg->acceleration.x;
            des.a.y() = msg->acceleration.y;
            des.a.z() = msg->acceleration.z;
            des.yaw = msg->yaw;
            des.yaw_rate = msg->yaw_dot;
            get_planning_msg = true;
        }
        m_controller.unlock();
    }

    void depthCallback_camera(const sensor_msgs::Image::ConstPtr&msg)
    {
        cv_bridge::CvImagePtr Dest ;
        Dest = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);

        depth_pic_camera = Dest->image;
        //cout<<"Output some info about the depth image in cv format"<<endl;
        //cout<< "Rows of the depth iamge = "<<depth_pic.rows<<endl;                       //获取深度图的行数height
        //cout<< "Cols of the depth iamge = "<<depth_pic.cols<<endl;                           //获取深度图的列数width
        //cout<< "Type of depth_pic's element = "<<depth_pic.type()<<endl;             //深度图的类型
        ushort d = depth_pic_camera.at<ushort>(depth_pic_camera.rows/2,depth_pic_camera.cols/2);           //读取深度值，数据类型为ushort单位为ｍｍ
        d_value_camera = float(d)/1000 ;      //强制转换
        cout<< "Value1 of depth_pic's pixel= "<<d_value_camera<<endl;    //读取深度值
    }

    void vins_cb(const nav_msgs::Odometry::ConstPtr& msg){
        vins_pose = *msg;

        Eigen::Isometry3d T_w_i = Eigen::Isometry3d::Identity(); //world系下imu的位姿（可以是cam的imu，也可以是px4的imu）
        //T_w_i的位置被转成了FLU
        T_w_i.rotate(Eigen::Quaterniond(Eigen::Vector4d(vins_pose.pose.pose.orientation.x, vins_pose.pose.pose.orientation.y, vins_pose.pose.pose.orientation.z, vins_pose.pose.pose.orientation.w)).toRotationMatrix());
        T_w_i.pretranslate(Eigen::Vector3d (vins_pose.pose.pose.position.x, vins_pose.pose.pose.position.y, vins_pose.pose.pose.position.z));
        Eigen::Isometry3d T_i_w = T_w_i.inverse();

        //vins位姿到px4位姿转换
        local_pose.header.frame_id = "odom";
        local_pose.child_frame_id = "base_link";
        local_pose.header.stamp = vins_pose.header.stamp;//
        // local_pose.header.stamp = ros::Time::now();
        // cout << "time error: " << ros::Time::now().toSec() - vins_pose.header.stamp.toSec() << endl;

        //位置
        if(use_camIMU == 1){
            Vector4d p_w_fi = T_w_i * T_ci_fi.matrix().col(3); //T_w_i是T_w_ci
            local_pose.pose.pose.position.x = p_w_fi[0];
            local_pose.pose.pose.position.y = p_w_fi[1];
            local_pose.pose.pose.position.z = p_w_fi[2];
        }
        else{//使用px4的imu
            local_pose.pose.pose.position.x = vins_pose.pose.pose.position.x;
            local_pose.pose.pose.position.y = vins_pose.pose.pose.position.y;
            local_pose.pose.pose.position.z = vins_pose.pose.pose.position.z;
        }
        //速度
        if(use_camIMU == 1){
            Vector4d v_wi_w = Vector4d(vins_pose.twist.twist.linear.x, vins_pose.twist.twist.linear.y, vins_pose.twist.twist.linear.z, 0); //cam IMU系在world系下的速度在world系下的表示
            Vector4d v_wi_i =  T_i_w * v_wi_w;//cam IMU系在world坐标系下的速度在瞬时cam IMU系下的表示，此时构建了cam IMU系的局部速度

            m_px4_imu.lock();
            V_wci_ci <<  0.0,                         -px4_imu.angular_velocity.z, px4_imu.angular_velocity.y, v_wi_i[0], 
                         px4_imu.angular_velocity.z,   0.0,                       -px4_imu.angular_velocity.x, v_wi_i[1],
                        -px4_imu.angular_velocity.y,   px4_imu.angular_velocity.x, 0.0,                        v_wi_i[2],
                         0.0,                          0.0,                        0.0,                        0.0;
            m_px4_imu.unlock();

            Vector4d v_wci_ci_fi = V_wci_ci * T_ci_fi.matrix().col(3);//计算cam IMU速度系(cam imu系在world系下的速度在cam系下的表示）下飞控IMU的速度（旋转和平移都有）
            Vector4d v_wfi_fi = T_fi_ci * v_wci_ci_fi;//计算飞控IMU系在world坐标系下的速度在飞控IMU系下的表示

            local_pose.twist.twist.linear.x = v_wfi_fi[0];
            local_pose.twist.twist.linear.y = v_wfi_fi[1];
            local_pose.twist.twist.linear.z = v_wfi_fi[2];
        }
        else{//使用px4的imu
            Vector4d v_wi_w = Vector4d(vins_pose.twist.twist.linear.x, vins_pose.twist.twist.linear.y, vins_pose.twist.twist.linear.z, 0); //px4 IMU系在world系下的速度在world系下的表示
            Vector4d v_wi_i =  T_i_w * v_wi_w;//px4 IMU系在world坐标系下的速度在瞬时px4 IMU系下的表示，此时构建了px4 IMU系的局部速度
            local_pose.twist.twist.linear.x = v_wi_i[0];
            local_pose.twist.twist.linear.y = v_wi_i[1];
            local_pose.twist.twist.linear.z = v_wi_i[2];
        }
        //姿态
        if(use_camIMU == 1){
            tf::Quaternion qua_vins(vins_pose.pose.pose.orientation.x, vins_pose.pose.pose.orientation.y, vins_pose.pose.pose.orientation.z, vins_pose.pose.pose.orientation.w);
            qua_vins = qua_vins * q_vins2px4;
            local_pose.pose.pose.orientation.w = qua_vins.w();
            local_pose.pose.pose.orientation.x = qua_vins.x();
            local_pose.pose.pose.orientation.y = qua_vins.y();
            local_pose.pose.pose.orientation.z = qua_vins.z();
        }
        else{//使用px4的imu
            local_pose.pose.pose.orientation.w = vins_pose.pose.pose.orientation.w;
            local_pose.pose.pose.orientation.x = vins_pose.pose.pose.orientation.x;
            local_pose.pose.pose.orientation.y = vins_pose.pose.pose.orientation.y;
            local_pose.pose.pose.orientation.z = vins_pose.pose.pose.orientation.z;
        }
        //角速度
        m_px4_imu.lock();
        local_pose.twist.twist.angular.x = px4_imu.angular_velocity.x;
        local_pose.twist.twist.angular.y = px4_imu.angular_velocity.y;
        local_pose.twist.twist.angular.z = px4_imu.angular_velocity.z;
        m_px4_imu.unlock();

        m_current_state.lock();
        if(current_state.mode != "MANUAL"){
            local_pose_pub.publish(local_pose);
        }
        m_current_state.unlock();

        m_controller.lock();
        odom_data.v(0) = vins_pose.twist.twist.linear.x;
        odom_data.v(1) = vins_pose.twist.twist.linear.y;
        odom_data.v(2) = vins_pose.twist.twist.linear.z;
        m_controller.unlock();

        //相机位姿发布，imu到cam位姿转换
        Eigen::Isometry3d T_w_cam = Eigen::Isometry3d::Identity();
        T_w_cam = T_w_i * T_i_cam; //使用哪个imu都是通用的，T_i_cam会自动变，注意T_i_cam为vins使用的imu到cam的变换
        Eigen::Quaterniond q_temp = Eigen::Quaterniond(T_w_cam.rotation());
        Eigen::Vector3d    t_temp = T_w_cam.translation();
        camera_pose.header.frame_id = "world";
        camera_pose.header.stamp = vins_pose.header.stamp;
        camera_pose.pose.position.x = t_temp[0];
        camera_pose.pose.position.y = t_temp[1];
        camera_pose.pose.position.z = t_temp[2]; 
        camera_pose.pose.orientation.x = q_temp.x();
        camera_pose.pose.orientation.y = q_temp.y();
        camera_pose.pose.orientation.z = q_temp.z();
        camera_pose.pose.orientation.w = q_temp.w();
        ego_cam_pose_pub.publish(camera_pose);
    }

    void initialize();
    void interrupt(void);
    void target_set_process(void);
    void timer_callback(void);
    void showInfo(void);
    bool force_arm_disarm(bool arm);
    void reboot_FCU();
};

#endif
