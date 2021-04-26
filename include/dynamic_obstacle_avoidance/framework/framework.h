#pragma once

#include "dynamic_obstacle_avoidance/move/servo_move.h"
#include "dynamic_obstacle_avoidance/global_planner/global_planner.h"
#include "dynamic_obstacle_avoidance/local_planner/apf_local_planner_creator.h"

#include "dynamic_obstacle_avoidance/SetJointValue.h"
#include "dynamic_obstacle_avoidance/SetTargetPose.h"
#include "dynamic_obstacle_avoidance/Execute.h"

#include "dynamic_obstacle_avoidance/SetObstacle.h"

class Framework
{
public:
    Framework(ros::NodeHandle *nh);
    ~Framework();

    void setJointTarget(const std::vector<double> &joint);
    void setTargetPose(const geometry_msgs::PoseStamped &p, const std::string &eef);

    /**
     * @brief 执行避障的主要函数
     * @return 0 执行成功， -1 规划失败， -2 超出迭代次数, -3 非自然停止
    */
    int execute();

private:
    bool setJointTargetCB(dynamic_obstacle_avoidance::SetJointValue::Request &req, dynamic_obstacle_avoidance::SetJointValue::Response &rep);
    bool setTargetPoseCB(dynamic_obstacle_avoidance::SetTargetPose::Request &req, dynamic_obstacle_avoidance::SetTargetPose::Response &rep);
    bool executeCB(dynamic_obstacle_avoidance::Execute::Request &req, dynamic_obstacle_avoidance::Execute::Response &rep);
    void setObstacleCB(const dynamic_obstacle_avoidance::SetObstacleConstPtr &msg);

    bool servoSwitchController(bool on_off);

private:
    ros::NodeHandle *m_nh;
    moveit::planning_interface::MoveGroupInterface *m_move_group;
    GlobalPlanner *m_global_planner;
    ILocalPlanner *m_local_planner;
    IMove *m_move;

    ros::ServiceServer m_set_joint_target_ser;
    ros::ServiceServer m_set_target_pose_ser;
    ros::ServiceServer m_execute_ser;
    ros::Subscriber m_set_obstacle_sub;

    ros::ServiceClient m_switch_controller_clinet;

    std::string m_default_controller;
    std::string m_servo_controller;

    bool m_is_servo_controller;
};
