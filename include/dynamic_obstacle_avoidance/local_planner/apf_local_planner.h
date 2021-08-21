#pragma once

#include "dynamic_obstacle_avoidance/local_planner/local_planner_interface.h"
#include "dynamic_obstacle_avoidance/local_planner/robot_jacobian.h"
#include <list>
#include <sensor_msgs/JointState.h>

struct ApfParam //势场参数结构体
{
    int max_iter; //最大迭代次数             10000
    int iter;
    double safety_distance; //安全距离       0.3

    // 目标
    double target_zeta;     //1                       # *_zeta 增益，一个比例
    double target_alfa;     //10                    # *_alfa 对由末端位置速度转换得到的关节得到数值乘以的比例
    double target_alfa_rot; //0.5                *_alfa_rot对由末端姿态速度转换得到的关节得到数值乘以的比例

    // 工作场的影响因子
    double dist_att;        //10                         # *_att 从那个  python工程    引入的参数，构型空间？大概类似坡度
    double dist_att_config; //0.5

    // 误差
    double target_err;     //0.02                      # 允许位置误差  位置总误差finisPosition
    double target_err_ori; //0.017444444               # *_ori 指的是姿态  姿态总误差 finishOrien

    // 轨迹

    double trajectory_zeta;     //1.0
    double tra_alfa;            //1                 *_alfa 对由末端位置速度转换得到的关节得到数值乘以的比例
    double tra_alfa_rot;        //0.5        *_alfa_rot对由末端姿态速度转换得到的关节得到数值乘以的比例
    double tra_dist_att;        //1
    double tra_dist_att_config; //0.5

    //内外圈
    double global_trajectory_att_outer; //0.3
    double global_trajectory_att_inner; //0.08

    // 障碍物
    // link
    std::vector<double> obs_eta;
    // std::vector<double> obs_rep;
    std::vector<double> obs_alfa;
};

class ApfLocalPlanner : public ILocalPlanner
{
public:
    ApfLocalPlanner(ros::NodeHandle *nh, moveit::planning_interface::MoveGroupInterface *group);
    ~ApfLocalPlanner();

    int execute() override;

private:
    //      获取距离   四种不同参数格式的形式
    double getDistance(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2);
    double getDistance(const geometry_msgs::PoseStamped &pose1, const Eigen::Isometry3d &pose2);
    double getDistance(const geometry_msgs::PoseStamped &pose1, const std::vector<double> &pose2);
    double getDistance(const Eigen::Isometry3d &pose1, const std::vector<double> &pose2);

    //获取障碍物距离
    void getObstacleDistance();

    //是否有符合轨迹跟随器的点
    bool getNearbyPoint();

    void fill(const geometry_msgs::PoseStamped &pose, std::vector<double> &positin);

    //执行全规划的轨迹
    int executeGlobalTra();

    //执行局部规划的轨迹
    int executeApf();

    //获取所有的作用力
    int getJointForce();

    //获取全局轨迹跟随器的力
    int getTrajectoryForce();

    //获取目标跟随器的力
    int getTargetPoseForce();

    //获取障碍物的斥力
    int getObstacleForce();

    //获取
    int getLinkPose();

    //姿态误差
    void getOrienError(const geometry_msgs::PoseStamped &p, const Eigen::Isometry3d &p2, Eigen::Vector3d &euler_angle);
    Eigen::Vector3d getOrienError(const geometry_msgs::PoseStamped &p, const Eigen::Isometry3d &p2);

    //位置总误差
    double finishPosition(const geometry_msgs::PoseStamped &s, const Eigen::Isometry3d &t);
    //姿态总误差
    double finishOrien(const geometry_msgs::PoseStamped &s, const Eigen::Isometry3d &t);
    //角度总误差     //很大？
    double finishJoint(std::vector<double> current_joint, std::vector<double> target_joint);

    // std::vector<double> getVelocity(std::vector<double> current_joint, const std::vector<double>& target_joint, std::vector<double> current_joint_velocity);

    //设置参数
    void setParam();

    void JointStateCB(const sensor_msgs::JointStateConstPtr &msg);

    void setTrajectoryIndex(int index, std::list<Eigen::Isometry3d>::iterator &iter);

    //速度转移矩阵
    void tr2jac(const Eigen::Isometry3d &T);

    //获取了两个link的转换矩阵
    Eigen::Isometry3d getTransform(const std::string &target_link, const std::string &source_link);

    double getRandon(double a, double b); //随机数

    std::vector<double> m_current_joint_value;    //当前关节
    std::vector<double> m_current_joint_velocity; //当前关节速度

    bool IK(const geometry_msgs::PoseStamped &p, std::vector<double> &j);

private:
    ApfParam *m_apf;

    // 迭代器
    // 使用方法iterator()要求容器返回一个Iterator。Iterator将准备好返回序列的第一个元素；
    std::list<Eigen::Isometry3d>::iterator m_tcp_trajectory_iter; //？

    int m_trajectory_point_index;

    //所有的力矩
    Eigen::Matrix<double, 6, 1> m_joint_att_force_p; //末端关节吸引力？
    Eigen::Matrix<double, 6, 1> m_joint_att_force_w;
    Eigen::Matrix<double, 6, 1> m_joint_att_trajectory_p; //轨迹吸引力？
    Eigen::Matrix<double, 6, 1> m_joint_att_trajectory_w;
    Eigen::Matrix<double, 6, 1> m_joint_rep_force; //关节斥力repulsive
    Eigen::Matrix<double, 6, 1> m_all_force;

    Eigen::Matrix<double, 6, 6> m_volecity_transform;

    std::vector<std::string> m_link_name;
    std::vector<geometry_msgs::PoseStamped> m_link_pose_vec; //区别？
    geometry_msgs::PoseStamped m_current_target_link_pose;

    Eigen::Matrix<double, 3, 1> m_current_p; //当前
    std::vector<double> m_current_w;

    RobotJacobian *m_robot_jacobian;

    std::vector<Eigen::MatrixXd> m_jacobian_p;
    Eigen::MatrixXd m_target_jacobian_w;
    Eigen::MatrixXd m_target_jacobian_p;
    std::vector<Eigen::MatrixXd> m_all_jacobian;

    ros::Subscriber m_joint_state_sub;

    bool m_is_stop;
};