#ifndef _TRAJ_REPLAN_H
#define _TRAJ_REPLAN_H

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include <fstream>
#include <ros/package.h>
#include <ros/ros.h>
#include <iostream>
#include <traj_utils/AssignOdom.h>
#include <traj_utils/IdealForm.h>
#include <traj_utils/ReachFlag.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <algorithm>

namespace ego_planner{

    class GRLR{

        public:
            GRLR() {}
            ~GRLR();
            void init(ros::NodeHandle &nh);
            bool CalculateAssignment(const std::vector<Eigen::Vector3d> &position_lg, const std::vector<Eigen::Vector3d> &desired_q, 
                                    std::vector<Eigen::Vector3d> &position_lg_star);
            void CalculateAlignment(int n);
            void RemapLocalGoals();
            bool allOnes(const Eigen::VectorXd& v);
            double getEsim();
            bool CallGlobalRemap();
        private:
            ros::NodeHandle node_;
            ros::Timer checkideal_timer_;
            ros::Subscriber assignodom_sub_, reach_ideal_sub_;
            ros::Publisher assignodom_pub_, idealform_pub_, reach_ideal_pub_;
            double swarm_relative_pts_[50][3];
            Eigen::Vector3d swarm_bias;
            std::vector<Eigen::Vector3d> swarm_pos, swarm_pos_star;
            std::vector<Eigen::Vector3d> desired_pos, vertical_pos, rotate_pos, across_pos;
            std::vector<int> sub_flag, reach_flag; 
            int form_type_;
            bool checkideal_flag_;

            bool all_flag(const std::vector<int> &form_flag, int type_);
        
            void AssignOdomCallback(const traj_utils::AssignOdomConstPtr &msg);
            void reachidealCallback(const traj_utils::ReachFlagConstPtr &msg);
            void checkidealCallback(const ros::TimerEvent &e);
            Eigen::VectorXd Calculate_w(Eigen::VectorXd ca);
            Eigen::Vector3d computeAverage(const std::vector<Eigen::Vector3d>& q);
            void Assign_matrix2Vector(const Eigen::MatrixXi &assign_best, std::vector<int> &assign_best_v, 
                                    const std::vector<Eigen::Vector3d> &desired_form, std::vector<Eigen::Vector3d> &assign_form);
        
        public:
            typedef std::shared_ptr<GRLR> Ptr;

            
            
    };

}

#endif