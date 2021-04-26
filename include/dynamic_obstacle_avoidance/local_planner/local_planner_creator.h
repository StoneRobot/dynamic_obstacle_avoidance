#pragma once
#include "dynamic_obstacle_avoidance/local_planner/local_planner_interface.h"

class LocalPlannerCreator
{
public:
    virtual ~LocalPlannerCreator(){}
    virtual ILocalPlanner* createLocalPlanner(ros::NodeHandle* nh ,moveit::planning_interface::MoveGroupInterface* group) = 0;
};