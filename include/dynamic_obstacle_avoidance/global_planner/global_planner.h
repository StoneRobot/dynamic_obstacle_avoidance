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
    
    // setJointTarget设置关节坐标
    void setJointTarget(const std::vector<double>& joint);
    //setTargetPose 设置笛卡尔坐标
    void setTargetPose(const geometry_msgs::PoseStamped& target_pose, const std::string& link_name="");
    
    //plan 进行全局规划
    bool plan();

   //获取目标姿态的笛卡尔坐标，局部规划器只能使用笛卡尔坐标
    void getTargetPose(Eigen::Isometry3d& target_pose, std::string& link_name);
    void getTrajectory(moveit_msgs::RobotTrajectory& joint_tra, std::list<Eigen::Isometry3d>& tcp_tra);

private:
    void FK(const std::vector<double>& joint, Eigen::Isometry3d& tcp);
    void joint_2_tcp();

private:
    moveit::planning_interface::MoveGroupInterface* m_move_group;   //运动规划接口
    geometry_msgs::PoseStamped m_target_pose;        //ROS消息的一种，参数是位姿   position and orientation/quaternion
    Eigen::Isometry3d m_target_pose_e;               //构造矩阵
    std::string m_eef;               
    moveit_msgs::RobotTrajectory m_tra;              //笛卡尔空间下的轨迹
    std::list<Eigen::Isometry3d> m_tcp_trajecory;

    //规划到一个关节空间内的目标
    //创建一个引用当前robot状态的指针。RobotState是包含所有当前位置/速度/加速度数据的对象
    moveit::core::RobotStatePtr m_robot_state_ptr;    
    
    const moveit::core::JointModelGroup* m_model;
};