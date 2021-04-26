#pragma once

#include "dynamic_obstacle_avoidance/move/move_interface.h"

class ServoMove : public IMove
{
public:
    ServoMove(ros::NodeHandle* nh);
    ~ServoMove();

    bool moveJointServo(const control_msgs::JointJog& jog) override;
    bool moveCartesian(const geometry_msgs::TwistStamped& twist) override;
    bool moveJointTarget(const std::vector<double>& joint) override;
    bool moveCartesianTarget(const geometry_msgs::PoseStamped& pose) override;
private:
    bool tips();

    

    ros::NodeHandle* m_nh;

    ros::Publisher m_joint_serve_pub;
    ros::Publisher m_twist_serve_pub;


};


