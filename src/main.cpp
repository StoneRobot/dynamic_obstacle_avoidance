#include "dynamic_obstacle_avoidance/framework/framework.h"

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "dynamic_obstacle_avoidance");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spin(4);
    spin.start();
    Framework f(&nh);
    ros::waitForShutdown();
    return 0;
}
