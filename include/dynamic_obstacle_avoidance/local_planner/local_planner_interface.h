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

   //设置全局规划器规划出的轨迹
    void setTragectory(const moveit_msgs::RobotTrajectory &trajectory, const std::list<Eigen::Isometry3d> &tcp_tar)
    {
        m_trajectory = trajectory;
        m_tcp_trajectory = tcp_tar;
    }

   //设置障碍物的位置和影响半径？
    void setObstacle(const std::vector<std::vector<double>> &obs_pose)
    {
        m_obs_pose = obs_pose;
    }

    //设置目标点的笛卡尔坐标      2个         数据格式不同↓
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

   //执行机器人动作
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