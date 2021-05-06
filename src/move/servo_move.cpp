#include "dynamic_obstacle_avoidance/move/servo_move.h"
using namespace std;

ServoMove::ServoMove(ros::NodeHandle *nh)
    : m_nh{nh}
{
    m_joint_serve_pub = m_nh->advertise<control_msgs::JointJog>("/servo_server/delta_joint_cmds", 10);
    m_twist_serve_pub = m_nh->advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 10);

}

ServoMove::~ServoMove()
{
}

bool ServoMove::moveJointServo(const control_msgs::JointJog &jog)
{
    m_joint_serve_pub.publish(jog);
    return true;
}

bool ServoMove::moveCartesian(const geometry_msgs::TwistStamped &twist)
{
    m_twist_serve_pub.publish(twist);
    return true;
}

bool ServoMove::moveJointTarget(const std::vector<double> &joint)
{
    return tips();
}

bool ServoMove::moveCartesianTarget(const geometry_msgs::PoseStamped &pose)
{
    return tips();
}

bool ServoMove::tips()
{
    ROS_INFO("To be continued...");
    return false;
}

