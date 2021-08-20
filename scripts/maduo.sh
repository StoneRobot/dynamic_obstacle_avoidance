while [ 1 ]; do
    rosrun dynamic_obstacle_avoidance set_obstacle.sh 
    rosrun dynamic_obstacle_avoidance set_joint1.sh
    rosservice call /dynamic_obstacle_avoidance/execute "{}"
    rosrun dynamic_obstacle_avoidance set_joint3.sh
    rosservice call /dynamic_obstacle_avoidance/execute "{}"
done
