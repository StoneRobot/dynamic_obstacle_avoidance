#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <string>

class RobotJacobian
{
public:
    RobotJacobian(ros::NodeHandle* nh, moveit::planning_interface::MoveGroupInterface *group, const std::vector<std::string>& link);
    ~RobotJacobian();

    bool getJacobian(Eigen::MatrixXd& jacobian, const std::string& frame);
    bool getAllJacobian(std::vector<Eigen::MatrixXd>& jacobians);
private:
    ros::NodeHandle* m_nh;
    moveit::planning_interface::MoveGroupInterface *m_move_group;

    std::vector<std::string> m_joint_link;
    std::string m_arm;

    robot_state::RobotStatePtr current_state;
    robot_model::RobotModelConstPtr model;
    const robot_model::JointModelGroup *j_model_group;
};



