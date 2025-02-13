#include <traj_replan/traj_replan.h>

//#include <traj_utils/LocalGoal.h>
//#include <traj_utils/SingleGoal.h>
//#include <quadrotor_msgs/PositionId.h>
//#include <quadrotor_msgs/Remap.h>
//#include <traj_utils/SimilarityError.h>

using std::vector;


namespace ego_planner
{
    GRLR::~GRLR()
    {

    }
    void GRLR::init(ros::NodeHandle &nh)
    {
        form_type_ = 1;
        checkideal_flag_ = false;

        swarm_pos.resize(6);
        sub_flag.resize(6);
        reach_flag.resize(6);

        for (int i = 0; i < 6; i++)
        {
            Eigen::Vector3d rel_pos;
            nh.param("global_goal/relative_pos_" + std::__cxx11::to_string(i) + "/x", rel_pos(0), -1.0);
            nh.param("global_goal/relative_pos_" + std::__cxx11::to_string(i) + "/y", rel_pos(1), -1.0);
            nh.param("global_goal/relative_pos_" + std::__cxx11::to_string(i) + "/z", rel_pos(2), -1.0);
            across_pos.push_back(rel_pos);
        }
        for (int i = 0; i < 6; i++)
        {
            Eigen::Vector3d rel_pos;
            nh.param("global_goal/rotate_pos_" + std::__cxx11::to_string(i) + "/x", rel_pos(0), -1.0);
            nh.param("global_goal/rotate_pos_" + std::__cxx11::to_string(i) + "/y", rel_pos(1), -1.0);
            nh.param("global_goal/rotate_pos_" + std::__cxx11::to_string(i) + "/z", rel_pos(2), -1.0);
            rotate_pos.push_back(rel_pos);
        }
        for (int i = 0; i < 6; i++)
        {
            Eigen::Vector3d rel_pos;
            nh.param("global_goal/vertical_pos_" + std::__cxx11::to_string(i) + "/x", rel_pos(0), -1.0);
            nh.param("global_goal/vertical_pos_" + std::__cxx11::to_string(i) + "/y", rel_pos(1), -1.0);
            nh.param("global_goal/vertical_pos_" + std::__cxx11::to_string(i) + "/z", rel_pos(2), -1.0);
            vertical_pos.push_back(rel_pos);
        }
            
        checkideal_timer_ = nh.createTimer(ros::Duration(1.0), &GRLR::checkidealCallback, this);

        assignodom_sub_ = nh.subscribe("/assignodom_sub", 20, &GRLR::AssignOdomCallback, this);
        assignodom_pub_ = nh.advertise<traj_utils::AssignOdom>("/assignodom_pub", 10);
        idealform_pub_ = nh.advertise<traj_utils::IdealForm>("/replan_idealform", 10);
        reach_ideal_sub_ = nh.subscribe("/drone_reach_ideal", 10, &GRLR::reachidealCallback, this);
        reach_ideal_pub_ = nh.advertise<std_msgs::Bool>("/drone_reach_all", 1);
    }

    bool GRLR::all_flag(const std::vector<int> &form_flag, int type_)
    {
            // 处理空vector的特殊情况（根据需求可选）
        if (form_flag.empty()) {
            // return 0; // 如果认为空vector不符合"所有元素等于type_"的条件
            return 0;   // 如果空vector视为满足条件（数学意义上的全称量词）
        }

        // 遍历所有元素检查
        for (int num : form_flag) {
            if (num != type_) {
                return 0; // 发现不符合立即返回0
            }
        }
        return 1; // 全部符合条件返回1
    }

    void GRLR::checkidealCallback(const ros::TimerEvent &e)
    {
        if(checkideal_flag_)
        {
            checkideal_flag_ = false;
            std_msgs::Bool msg1;
            msg1.data = true;
            reach_ideal_pub_.publish(msg1);
        }
    }

    void GRLR::reachidealCallback(const traj_utils::ReachFlagConstPtr &msg)
    {
        if(msg->drone_id > reach_flag.size())
            return;
        reach_flag[msg->drone_id] = msg->form_type;
        if (all_flag(reach_flag, 2) || all_flag(reach_flag, 3))
        {
            checkideal_flag_ = true;
        }
    }

    void GRLR::AssignOdomCallback(const traj_utils::AssignOdomConstPtr &msg)
    {
        /*
        if(msg->num >= swarm_pos.size())
        {
            swarm_pos.resize(msg->num + 1);
            sub_flag.resize(msg->num + 1);
        }
        */
        Eigen::Vector3d temp;
        temp << msg->x, msg->y, msg->z;
        swarm_pos[msg->num] = temp;
        sub_flag[msg->num] = msg->formation_type;
        if (sub_flag.size() == 6)
        {
            //ROS_WARN("Go in assignodomCallback!!!");
            if (all_flag(sub_flag, form_type_))
            {
                form_type_++;
                ROS_WARN("check assignodomCallback success!!!");
                //sub_flag.clear();
                swarm_bias = computeAverage(swarm_pos);
                ROS_ERROR("average is %lf, %lf, %lf", swarm_bias[0], swarm_bias[1], swarm_bias[2]);
                switch (form_type_)
                {
                case 1:
                    desired_pos.clear();
                    desired_pos = across_pos;
                    break;
                
                case 2:
                    desired_pos.clear();
                    desired_pos = rotate_pos;
                    break;

                case 3:
                    desired_pos.clear();
                    desired_pos = vertical_pos;
                    break;

                default:
                    break;
                }
                bool success = CalculateAssignment(swarm_pos, desired_pos, swarm_pos_star);
                ROS_WARN("Swarm pos star size is %d!!!", swarm_pos_star.size());
                ROS_WARN("Change Success!!!");
            }
            else 
                return;
            traj_utils::AssignOdom msg1;
            traj_utils::IdealForm msg2;
            for (const auto& vec : swarm_pos_star)
            {
                geometry_msgs::Point point;
                point.x = vec(0);
                point.y = vec(1);
                point.z = vec(2);
                msg2.form.push_back(point);
            }
            //msg2.form = swarm_pos_star;
            msg2.form_type = form_type_;
            idealform_pub_.publish(msg2);
            for (int i = 0; i < swarm_pos_star.size(); i++)
            {
                if (form_type_ == 2)
                {
                    msg1.x = swarm_pos_star[i][0] + swarm_bias(0)-2.5;
                    msg1.y = swarm_pos_star[i][1] + swarm_bias(1);
                    msg1.z = swarm_pos_star[i][2] + swarm_bias(2);
                }
                else if(form_type_ == 3)
                {
                    msg1.x = swarm_pos_star[i][0] + swarm_bias(0)-3.0;
                    msg1.y = swarm_pos_star[i][1];
                    msg1.z = swarm_pos_star[i][2] + swarm_bias(2);
                }
                
                //msg1.x = swarm_pos_star[i][0];
                //msg1.y = swarm_pos_star[i][1];
                //msg1.z = swarm_pos_star[i][2];
                msg1.num = i;
                msg1.formation_type = form_type_;
                assignodom_pub_.publish(msg1);
            }
        }
    }

    /*
    Eigen::VectorXd GRLR::Calculate_w(Eigen::VectorXd ca)
    {
        Eigen::VectorXd exp_ca = ca.array().exp();
        double sum = exp_ca.sum();
        return exp_ca;
    }
    */

    Eigen::Vector3d GRLR::computeAverage(const std::vector<Eigen::Vector3d>& q) 
    {
        if (q.empty()) {
            // 返回零向量或根据需求处理异常
            return Eigen::Vector3d::Zero();
        }
        
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for (const auto& vec : q) {
            sum += vec;
        }
        return sum / static_cast<double>(q.size());
    }


    bool GRLR::CalculateAssignment(const std::vector<Eigen::Vector3d> &position_lg, const std::vector<Eigen::Vector3d> &desired_q, 
                                    std::vector<Eigen::Vector3d> &position_lg_star)
    {
        if (position_lg.size() != desired_q.size())
            return false;
        position_lg_star.clear();
        int n = position_lg.size();
        Eigen::MatrixXi matrix = Eigen::MatrixXi::Zero(n, n);
        std::vector<int> indices(n);
        double kij = 0;
        double assign_cost, assign_cost_best;
        Eigen::MatrixXi assign_best;//最优的队形分配o
        assign_cost = 0;

        // 初始化indices为0, 1, ..., n - 1
        for (int i = 0; i < n; ++i) indices[i] = i;

        // 遍历所有排列
        do {
            // 根据当前排列，构建排列矩阵
            for (int i = 0; i < n; i++) {
                matrix(i, indices[i]) = 1;
            }
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    if (matrix(i, j)) {
                        kij = position_lg[i].transpose().dot(desired_q[j]);
                        assign_cost -= kij;
                    }
                }
            }
            
            if (assign_cost < assign_cost_best) {
                assign_cost_best = assign_cost;
                assign_best = matrix;
            }
            
            // 重置矩阵为零以准备下一个排列
            matrix.setZero();

        } while (std::next_permutation(indices.begin(), indices.end()));
        std::vector<int> assign_v;
        Assign_matrix2Vector(assign_best, assign_v, desired_q, position_lg_star);
        //std::cout << assign_best << "\n\n";
        return true;
    }

    /*
    void GRLR::CalculateAlignment(int n)
    {
        blg = 0;
        bq = 0;
        blgq = 0;
        for (int i = 0; i < n; i++)
        {
            blg += weight(i) * position_lg[i].transpose().dot(position_lg[i]);
            bq += weight(i) * desired_q[i].transpose().dot(desired_q[i]);
            blgq += weight(i) * position_lg[i].transpose().dot(desired_q[i]);
            q_hat += weight(i) * desired_q[i];
            lg_hat += weight(i) * position_lg[i];
        }
        s_star = (blgq - lg_hat.transpose().dot(q_hat)) / (bq - q_hat.transpose().dot(q_hat));
        d_star = lg_hat - s_star * q_hat;
    }
    */

    void GRLR::Assign_matrix2Vector(const Eigen::MatrixXi &assign_best, std::vector<int> &assign_best_v, 
                                    const std::vector<Eigen::Vector3d> &desired_form, std::vector<Eigen::Vector3d> &assign_form)
    {
        assign_best_v.resize(6, 0);
        assign_form.clear();
        // 遍历矩阵以填充assignment向量
        ROS_WARN("assign_best.row is %d", assign_best.rows());
        for (size_t i = 0; i < assign_best.rows(); ++i) 
        {
            for (size_t j = 0; j < assign_best.cols(); ++j) 
            {
                if (assign_best(i, j) == 1) {
                    // 如果在(i, j)位置找到1，表示第i个人分配到第j个任务
                    assign_best_v[i] = j;
                    Eigen::Vector3d temp = desired_form[j];
                    assign_form.push_back(temp);
                    ROS_WARN("Size is %d", assign_form.size());
                    break; // 跳出内层循环
                }
            }
        }
    }
    //assign_best_v s_star d_star
    /*
    void GRLR::RemapLocalGoals()
    {
        position_lg_star.clear();
        for (int i = 0; i < assign_best_v.norm(); i++)
        {
            position_lg_star[i] = s_star * q_r[assign_best_v[i]] + d_star;
        }
    }

    
    bool GRLR::allOnes(const Eigen::VectorXd& v) {
        return (v.array() == 1).all();
    }
    */
}

using namespace ego_planner;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trj_replan_node");
    ros::NodeHandle nh("~");

    GRLR grlr_opt_;

    grlr_opt_.init(nh);

    // ros::Duration(1.0).sleep();
    ros::spin();

    return 0;

}


