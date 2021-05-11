#include "dynamic_obstacle_avoidance/framework/framework.h"
#include <iostream>
#include <controller_manager_msgs/SwitchController.h>
#include <vector>
#include <set>
#include <visualization_msgs/MarkerArray.h>
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
    m_pub_obs_mark_array_pub = m_nh->advertise<visualization_msgs::MarkerArray>("visualization_marker2", 100);
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
    // 进行局部规划
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
    static std::vector<int> obs_name;
    visualization_msgs::MarkerArray remove_mark;
    remove_mark.markers.resize(1);
    remove_mark.markers[0].action = remove_mark.markers[0].DELETEALL;
    m_pub_obs_mark_array_pub.publish(remove_mark);

    // set<vector<double>> pose_set;
    vector<vector<double>> pose;
    pose.resize(msg->pose.size());
    visualization_msgs::MarkerArray mark_array_msgs;
    mark_array_msgs.markers.resize(msg->pose.size());
    for (size_t i = 0; i < pose.size(); i++)
    {
        pose[i].resize(3);
        // pose_set.insert()
        mark_array_msgs.markers[i].header.frame_id = "world";
        mark_array_msgs.markers[i].id = i;
        mark_array_msgs.markers[i].type = mark_array_msgs.markers[i].SPHERE;
        mark_array_msgs.markers[i].action = mark_array_msgs.markers[i].ADD;
        mark_array_msgs.markers[i].pose.position.x = msg->pose[i].data[0];
        mark_array_msgs.markers[i].pose.position.y = msg->pose[i].data[1];
        mark_array_msgs.markers[i].pose.position.z = msg->pose[i].data[2];
        mark_array_msgs.markers[i].scale.x = 0.05;
        mark_array_msgs.markers[i].scale.y = 0.05;
        mark_array_msgs.markers[i].scale.z = 0.05;
        mark_array_msgs.markers[i].color.a = 1;
        mark_array_msgs.markers[i].color.r = 1;
        for (size_t j = 0; j < 3; j++)
        {
            pose[i][j] = msg->pose[i].data[j];
        }
    }
    m_pub_obs_mark_array_pub.publish(mark_array_msgs);
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