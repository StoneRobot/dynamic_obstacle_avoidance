#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <string>
#include <list>

class GlobalPlanner
{
public:
    GlobalPlanner(moveit::planning_interface::MoveGroupInterface* move_group);
    ~GlobalPlanner();

    void setJointTarget(const std::vector<double>& joint);
    void setTargetPose(const geometry_msgs::PoseStamped& target_pose, const std::string& link_name="");
    bool plan();
    void getTargetPose(Eigen::Isometry3d& target_pose, std::string& link_name);
    
    void getTrajectory(moveit_msgs::RobotTrajectory& joint_tra, std::list<Eigen::Isometry3d>& tcp_tra);

private:
    void FK(const std::vector<double>& joint, Eigen::Isometry3d& tcp);
    void joint_2_tcp();

private:
    moveit::planning_interface::MoveGroupInterface* m_move_group;
    geometry_msgs::PoseStamped m_target_pose;
    Eigen::Isometry3d m_target_pose_e;
    std::string m_eef;
    moveit_msgs::RobotTrajectory m_tra;
    std::list<Eigen::Isometry3d> m_tcp_trajecory;

    moveit::core::RobotStatePtr m_robot_state_ptr;
    const moveit::core::JointModelGroup* m_model;
};


