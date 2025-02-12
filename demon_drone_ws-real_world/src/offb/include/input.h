#ifndef __INPUT_H
#define __INPUT_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseStamped.h>
#include "PX4CtrlParam.h"
#include <nav_msgs/Odometry.h>

template <typename Scalar_t>
Scalar_t normalize_angle(Scalar_t a) {
    int cnt = 0;
    while (true) {
        cnt++;

        if (a < -M_PI) {
            a += M_PI * 2.0;
        } else if (a > M_PI) {
            a -= M_PI * 2.0;
        }

        if (-M_PI <= a && a <= M_PI) {
            break;
        };

        assert(cnt < 10 && "[uav_utils/geometry_msgs] INVALID INPUT ANGLE");
    }

    return a;
}

inline void extract_odometry_gps(nav_msgs::OdometryConstPtr msg, Eigen::Vector3d& p,
                                Eigen::Vector3d& v, Eigen::Quaterniond& q)
{
    // p(0) = - ( msg->pose.position.x - p_init(0) );
    // p(1) = - ( msg->pose.position.y - p_init(1) );
    // p(2) = msg->pose.position.z - p_init(2);

    p(0) = - msg->pose.pose.position.x;
    p(1) = - msg->pose.pose.position.y;
    p(2) = msg->pose.pose.position.z;

    // v(0) = (p(0) - p_l(0)) / dur;
    // v(1) = (p(1) - p_l(1)) / dur;
    // v(2) = (p(2) - p_l(2)) / dur;

    v(0) = msg->twist.twist.linear.x;
    v(1) = msg->twist.twist.linear.y;
    v(2) = msg->twist.twist.linear.z;

    // p_l = p;

    q.w() = msg->pose.pose.orientation.w;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
}

inline void extract_odometry(nav_msgs::OdometryConstPtr msg, Eigen::Vector3d& p,
                            Eigen::Vector3d& v, Eigen::Quaterniond& q)
{
    p(0) = msg->pose.pose.position.x;
    p(1) = msg->pose.pose.position.y;
    p(2) = msg->pose.pose.position.z;

    // v(0) = (p(0) - p_l(0)) / dur;
    // v(1) = (p(1) - p_l(1)) / dur;
    // v(2) = (p(2) - p_l(2)) / dur;

    v(0) = msg->twist.twist.linear.x;
    v(1) = msg->twist.twist.linear.y;
    v(2) = msg->twist.twist.linear.z;

    // p_l = p;

    q.w() = msg->pose.pose.orientation.w;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
}

inline void extract_odometry(nav_msgs::OdometryConstPtr msg, Eigen::Vector3d& p, 
                            Eigen::Vector3d& v, Eigen::Quaterniond& q, Eigen::Vector3d& w, int o_s)
{
    if(o_s == 1)
        extract_odometry_gps(msg, p, v, q);
    else
        extract_odometry(msg, p, v, q);

    w(0) = msg->twist.twist.angular.x;
    w(1) = msg->twist.twist.angular.y;
    w(2) = msg->twist.twist.angular.z;
    // w(0) = 0;
    // w(1) = 0;
    // w(2) = 0;
}

class RC_Data_t
{
public:
  double mode;
  double gear;
  double reboot_cmd;
  double last_mode;
  double last_gear;
  double last_reboot_cmd;
  bool have_init_last_mode{false};
  bool have_init_last_gear{false};
  bool have_init_last_reboot_cmd{false};
  double ch[4];

  mavros_msgs::RCIn msg;
  ros::Time rcv_stamp;

  bool is_command_mode;
  bool enter_command_mode;
  bool is_hover_mode;
  bool enter_hover_mode;
  bool toggle_reboot;

  static constexpr double GEAR_SHIFT_VALUE = 0.75;
  static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;
  static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;
  static constexpr double DEAD_ZONE = 0.25;

  RC_Data_t();
  void check_validity();
  bool check_centered();
  void feed(mavros_msgs::RCInConstPtr pMsg);
  bool is_received(const ros::Time &now_time);
};

class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  // Eigen::Vector3d p_l;
  // Eigen::Vector3d p_init;
  Eigen::Vector3d v;
  // Eigen::Vector3d v_ave;
  Eigen::Quaterniond q;
  Eigen::Vector3d w;

  // double l_t = 0;
  // double t_delta_0;
  int odom_source = 0; //0:mocap, 1:gps

  // int init_count = 0;
  // double first_odom_time;

  nav_msgs::Odometry msg;
  ros::Time rcv_stamp;
  bool recv_new_msg;

  Odom_Data_t();
  void feed(nav_msgs::OdometryConstPtr pMsg);
};

class Imu_Data_t
{
public:
  Eigen::Quaterniond q;
  Eigen::Vector3d w;
  Eigen::Vector3d a;

  sensor_msgs::Imu msg;
  ros::Time rcv_stamp;

  Imu_Data_t();
  void feed(sensor_msgs::ImuConstPtr pMsg);
};

class State_Data_t
{
public:
  mavros_msgs::State current_state;
  mavros_msgs::State state_before_offboard;

  State_Data_t();
  void feed(mavros_msgs::StateConstPtr pMsg);
};

class ExtendedState_Data_t
{
public:
  mavros_msgs::ExtendedState current_extended_state;

  ExtendedState_Data_t();
  void feed(mavros_msgs::ExtendedStateConstPtr pMsg);
};

class Command_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  double yaw;
  double yaw_rate;

  quadrotor_msgs::PositionCommand msg;
  ros::Time rcv_stamp;

  Command_Data_t();
  void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
};

class Battery_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double volt{0.0};
  double percentage{0.0};

  sensor_msgs::BatteryState msg;
  ros::Time rcv_stamp;

  Battery_Data_t();
  void feed(sensor_msgs::BatteryStateConstPtr pMsg);
};

class Takeoff_Land_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool triggered{false};
  uint8_t takeoff_land_cmd; // see TakeoffLand.msg for its defination

  quadrotor_msgs::TakeoffLand msg;
  ros::Time rcv_stamp;

  Takeoff_Land_Data_t();
  void feed(quadrotor_msgs::TakeoffLandConstPtr pMsg);
};

#endif
