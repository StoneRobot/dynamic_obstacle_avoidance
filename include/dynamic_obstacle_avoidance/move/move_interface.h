#pragma once
#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <control_msgs/JointJog.h>
#include <geometry_msgs/TwistStamped.h>

class IMove
{
public:
    virtual ~IMove(){}

    virtual bool moveJointServo(const control_msgs::JointJog& jog) = 0;
    
    virtual bool moveCartesian(const geometry_msgs::TwistStamped& twist) = 0;

    virtual bool moveJointTarget(const std::vector<double>& joint) = 0;

    virtual bool moveCartesianTarget(const geometry_msgs::PoseStamped& pose) = 0;
private:
    /* data */
};
