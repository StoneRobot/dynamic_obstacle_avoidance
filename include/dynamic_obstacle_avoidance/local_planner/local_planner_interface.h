#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <list>

#include "dynamic_obstacle_avoidance/move/move_interface.h"

class ILocalPlanner
{
public:
    ILocalPlanner(ros::NodeHandle *nh, moveit::planning_interface::MoveGroupInterface *group)
        : m_nh{nh},
          m_move_group{group}
    {
    }

    virtual ~ILocalPlanner() {}

    void setTragectory(const moveit_msgs::RobotTrajectory &trajectory, const std::list<Eigen::Isometry3d> &tcp_tar)
    {
        m_trajectory = trajectory;
        m_tcp_trajectory = tcp_tar;
    }

    void setObstacle(const std::vector<std::vector<double>> &obs_pose)
    {
        m_obs_pose = obs_pose;
    }

    void setTargetPose(const geometry_msgs::PoseStamped &target_pose, const std::string &eef)
    {
        m_target_pose = target_pose;
        m_eef = eef;
    }

    void setTargetPose(const Eigen::Isometry3d &target_pose, const std::string &eef)
    {
        m_target_pose_e = target_pose;
        m_eef = eef;
    }

    void setMoveInterface(IMove *move)
    {
        m_move = move;
    }

    virtual int execute() = 0;

protected:
    ros::NodeHandle *m_nh;
    moveit::planning_interface::MoveGroupInterface *m_move_group;

    geometry_msgs::PoseStamped m_target_pose;
    Eigen::Isometry3d m_target_pose_e;
    std::string m_eef;
    std::vector<std::vector<double>> m_obs_distance;
    std::vector<std::vector<double>> m_obs_pose;

    moveit_msgs::RobotTrajectory m_trajectory;
    std::list<Eigen::Isometry3d> m_tcp_trajectory;

    IMove *m_move;
};