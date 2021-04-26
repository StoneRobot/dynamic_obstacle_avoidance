#include "dynamic_obstacle_avoidance/framework/framework.h"
#include <iostream>
#include <controller_manager_msgs/SwitchController.h>
#include <vector>
#include <set>
using namespace std;

Framework::Framework(ros::NodeHandle *nh)
    : m_nh{nh}
{
    system("rosrun dynamic_obstacle_avoidance load_param.sh");
    string arm_name;
    m_nh->param<string>("framework/move_group", arm_name, "arm");
    m_move_group = new moveit::planning_interface::MoveGroupInterface(arm_name);
    m_global_planner = new GlobalPlanner(m_move_group);
    ApfLocalPlannerCreator apf_creator;
    m_local_planner = apf_creator.createLocalPlanner(m_nh, m_move_group);
    m_move = new ServoMove(m_nh);
    m_local_planner->setMoveInterface(m_move);

    m_set_joint_target_ser = m_nh->advertiseService("set_joint_value", &Framework::setJointTargetCB, this);
    m_set_target_pose_ser = m_nh->advertiseService("set_target_pose", &Framework::setTargetPoseCB, this);
    m_execute_ser = m_nh->advertiseService("execute", &Framework::executeCB, this);

    m_set_obstacle_sub = m_nh->subscribe("set_obstacle", 10, &Framework::setObstacleCB, this);
    m_switch_controller_clinet = m_nh->serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

    m_is_servo_controller = false;
    m_nh->param<string>("default_controller", m_default_controller, "arm_joint_controller");
    m_nh->param<string>("servo_controller", m_servo_controller, "joint_group_position_controller");
}

Framework::~Framework()
{
    delete m_move;
    delete m_local_planner;
    delete m_global_planner;
    delete m_move_group;
    m_move = nullptr;
    m_local_planner = nullptr;
    m_global_planner = nullptr;
    m_move_group = nullptr;
}

void Framework::setJointTarget(const std::vector<double> &joint)
{
    m_global_planner->setJointTarget(joint);
}

void Framework::setTargetPose(const geometry_msgs::PoseStamped &p, const string &eef)
{
    m_global_planner->setTargetPose(p);
}

int Framework::execute()
{
    int flag = 0;
    servoSwitchController(true);
    if (m_global_planner->plan() == 0)
    {
        cout << "全局规划失败" << endl;
        return -1;
    }

    Eigen::Isometry3d target_pose;
    string eef;
    m_global_planner->getTargetPose(target_pose, eef);
    m_local_planner->setTargetPose(target_pose, eef);

    moveit_msgs::RobotTrajectory tra;
    list<Eigen::Isometry3d> tcp_tra;
    m_global_planner->getTrajectory(tra, tcp_tra);
    m_local_planner->setTragectory(tra, tcp_tra);
    flag = m_local_planner->execute();
    servoSwitchController(false);
    return flag;
}

bool Framework::setJointTargetCB(dynamic_obstacle_avoidance::SetJointValue::Request &req, dynamic_obstacle_avoidance::SetJointValue::Response &rep)
{
    setJointTarget(req.joint_value);
    return true;
}

bool Framework::setTargetPoseCB(dynamic_obstacle_avoidance::SetTargetPose::Request &req, dynamic_obstacle_avoidance::SetTargetPose::Response &rep)
{
    setTargetPose(req.pose, req.eef);
    return true;
}

bool Framework::executeCB(dynamic_obstacle_avoidance::Execute::Request &req, dynamic_obstacle_avoidance::Execute::Response &rep)
{
    rep.code = execute();
    return true;
}

void Framework::setObstacleCB(const dynamic_obstacle_avoidance::SetObstacleConstPtr &msg)
{
    // set<vector<double>> pose_set;
    vector<vector<double>> pose;
    pose.resize(msg->pose.size());
    for (size_t i = 0; i < pose.size(); i++)
    {
        pose[i].resize(3);
        // pose_set.insert()
        for (size_t j = 0; j < 3; j++)
        {
            pose[i][j] = msg->pose[i].data[j];
        }
    }
    m_local_planner->setObstacle(pose);
}

bool Framework::servoSwitchController(bool on_off)
{
    controller_manager_msgs::SwitchController srv;
    if (on_off && !m_is_servo_controller)
    {
        srv.request.start_controllers.push_back(m_servo_controller);
        srv.request.stop_controllers.push_back(m_default_controller);
        m_is_servo_controller = true;
    }
    else if (!on_off && m_is_servo_controller)
    {
        srv.request.start_controllers.push_back(m_default_controller);
        srv.request.stop_controllers.push_back(m_servo_controller);
        m_is_servo_controller = false;
    }
    srv.request.strictness = 2;
    m_switch_controller_clinet.call(srv);
    return srv.response.ok;
}