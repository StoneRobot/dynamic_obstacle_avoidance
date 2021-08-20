#include "dynamic_obstacle_avoidance/global_planner/global_planner.h"

GlobalPlanner::GlobalPlanner(moveit::planning_interface::MoveGroupInterface *move_group)
    : m_move_group{move_group}
{
    m_robot_state_ptr = m_move_group->getCurrentState();
    m_model = m_robot_state_ptr->getJointModelGroup(m_move_group->getName());
}

//析构函数(destructor) 与构造函数相反，当对象结束其生命周期，如对象所在的函数已调用完毕时，
//系统自动执行析构函数。析构函数往往用来做“清理善后” 的工作（例如在建立对象时用new开辟
//了一片内存空间，delete会自动调用析构函数后释放内存）。
//析构函数，delete会自动调用析构函数释放内存
GlobalPlanner::~GlobalPlanner()
{
}

void GlobalPlanner::setJointTarget(const std::vector<double> &joint)
{
    m_move_group->setJointValueTarget(joint);
    m_eef = m_move_group->getEndEffectorLink();
    ROS_INFO_STREAM("m_eef: >>>>>>>>>>>>>>>" << m_eef) ;
    FK(joint, m_target_pose_e);
}

void GlobalPlanner::setTargetPose(const geometry_msgs::PoseStamped &target_pose, const std::string &link_name)
{
    m_move_group->setPoseTarget(target_pose, link_name);
    Eigen::Isometry3d p = Eigen::Isometry3d::Identity(); //矩阵初始化
    Eigen::Matrix<double, 3, 1> t;
    t << target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z;
    Eigen::Quaterniond q(target_pose.pose.orientation.w, target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z);
    p.pretranslate(t);
    p.prerotate(q);
    m_target_pose_e = p; // 4x4构造矩阵
    m_eef = link_name;
}

bool GlobalPlanner::plan()
{
    m_move_group->setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan p;
    if (m_move_group->plan(p) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        m_tra = p.trajectory_;
        joint_2_tcp();
        return true;
    }
    return false;
}

void GlobalPlanner::getTargetPose(Eigen::Isometry3d &target_pose, std::string &link_name)
{
    target_pose = m_target_pose_e;
    link_name = m_eef;
}

void GlobalPlanner::getTrajectory(moveit_msgs::RobotTrajectory &joint_tra, std::list<Eigen::Isometry3d> &tcp_tra)
{
    joint_tra = m_tra;
    tcp_tra = m_tcp_trajecory;
}

void GlobalPlanner::FK(const std::vector<double> &joint, Eigen::Isometry3d &tcp)
{
    m_robot_state_ptr->setJointGroupPositions(m_model, joint);
    tcp = m_robot_state_ptr->getGlobalLinkTransform(m_eef);
}

void GlobalPlanner::joint_2_tcp()
{
    m_tcp_trajecory.resize(m_tra.joint_trajectory.points.size());
    std::list<Eigen::Isometry3d>::iterator a = m_tcp_trajecory.begin();
    for (int i = 0; i < m_tcp_trajecory.size(); i++)
    {
        FK(m_tra.joint_trajectory.points[i].positions, *a);
        ++a;
    }
}