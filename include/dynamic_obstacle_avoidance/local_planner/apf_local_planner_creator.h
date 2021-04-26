#pragma once

#include "dynamic_obstacle_avoidance/local_planner/local_planner_creator.h"
#include "dynamic_obstacle_avoidance/local_planner/apf_local_planner.h"

class ApfLocalPlannerCreator : public LocalPlannerCreator
{
public:
    ILocalPlanner* createLocalPlanner(ros::NodeHandle* nh ,moveit::planning_interface::MoveGroupInterface* group) override;
};
