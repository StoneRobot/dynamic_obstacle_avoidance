#include "dynamic_obstacle_avoidance/local_planner/apf_local_planner_creator.h"


ILocalPlanner* ApfLocalPlannerCreator::createLocalPlanner(ros::NodeHandle* nh ,moveit::planning_interface::MoveGroupInterface* group)
{
    return new ApfLocalPlanner(nh, group);
}