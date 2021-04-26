#include "dynamic_obstacle_avoidance/local_planner/apf_local_planner.h"

#include <math.h>
#include <algorithm>
#include <time.h>
#include <tf/transform_listener.h>

using namespace std;

ApfLocalPlanner::ApfLocalPlanner(ros::NodeHandle *nh, moveit::planning_interface::MoveGroupInterface *group)
    : ILocalPlanner{nh, group},
      m_is_stop{false}
{
    vector<string> l{"link2", "link3", "link4", "link5", "link6", "pick_link"};
    m_nh->param<vector<string>>("link", m_link_name, l);
    m_jacobian_p.resize(m_link_name.size());
    m_robot_jacobian = new RobotJacobian(nh, m_move_group, m_link_name);
    m_apf = new ApfParam;

    m_joint_state_sub = m_nh->subscribe("/joint_states", 10, &ApfLocalPlanner::JointStateCB, this);

    setParam();
    Eigen::Isometry3d t = getTransform(m_link_name[m_link_name.size() - 2], m_link_name[m_link_name.size() - 1]);
    tr2jac(t);
}

ApfLocalPlanner::~ApfLocalPlanner()
{
    delete m_robot_jacobian;
    delete m_apf;
    m_robot_jacobian = nullptr;
}

int ApfLocalPlanner::execute()
{
    m_apf->iter = 0;
    m_trajectory_point_index = 1;
    m_tcp_trajectory_iter = m_tcp_trajectory.begin();
    ++m_tcp_trajectory_iter;
    int flag = 0;
    flag = executeGlobalTra();
    if (flag == -1)
        flag = executeApf();
    return flag;
}

int ApfLocalPlanner::executeGlobalTra()
{
    while (ros::ok() && !m_is_stop)
    {
        static int cnt = 0;
        getObstacleDistance();
        for (size_t i = 0; i < m_obs_distance.size(); i++)
            for (size_t j = 0; j < m_obs_distance[i].size(); j++)
                if (m_obs_distance[i][j] < m_apf->safety_distance)
                    return -1;

        control_msgs::JointJog jog;
        jog.header.frame_id = m_trajectory.joint_trajectory.header.frame_id;
        jog.joint_names = m_trajectory.joint_trajectory.joint_names;

        int index;
        m_trajectory_point_index == m_trajectory.joint_trajectory.points.size() - 1 ? index = m_trajectory_point_index - 1 : index = m_trajectory_point_index;
        jog.velocities.resize(6);
        for (int i = 0; i < 6; i++)
        {
            jog.velocities[i] = m_trajectory.joint_trajectory.points[index].velocities[i] * 0.3;
        }

        jog.duration = m_trajectory.joint_trajectory.points[m_trajectory_point_index].time_from_start.toSec() - m_trajectory.joint_trajectory.points[m_trajectory_point_index - 1].time_from_start.toSec();
        m_move->moveJointServo(jog);
        double err = finishJoint(m_current_joint_value, m_trajectory.joint_trajectory.points[m_trajectory_point_index].positions);
        if (err < 0.052333333)
        {
            ++m_trajectory_point_index;
            ROS_INFO_STREAM("m_trajectory_point_index: " << m_trajectory_point_index);
            ++m_tcp_trajectory_iter;
        }
        // ++cnt;
        if (m_trajectory_point_index >= m_trajectory.joint_trajectory.points.size())
            break;
    }
    return 0;
}

int ApfLocalPlanner::executeApf()
{
    int flag = 0;
    getLinkPose();
    m_current_target_link_pose = m_move_group->getCurrentPose(m_eef);
    double position_err = finishPosition(m_current_target_link_pose, m_target_pose_e);
    double orien_err = finishOrien(m_current_target_link_pose, m_target_pose_e);
    while (ros::ok() && !m_is_stop && (position_err > m_apf->target_err || orien_err > m_apf->target_err_ori))
    {
        clock_t t1 = clock();

        m_robot_jacobian->getAllJacobian(m_all_jacobian);
        vector<Eigen::MatrixXd> jacobian_p;
        jacobian_p.resize(m_link_name.size());

        for (int i = 0; i < jacobian_p.size(); ++i)
        {
            m_jacobian_p[i] = m_all_jacobian[i].block<6, 3>(0, 0);
        }
        // link6
        m_target_jacobian_w = m_all_jacobian[m_link_name.size() - 2].block<6, 3>(0, 3);
        m_target_jacobian_p = m_all_jacobian[m_link_name.size() - 2].block<6, 3>(0, 0);

        getLinkPose();
        m_current_target_link_pose = m_move_group->getCurrentPose(m_eef);
        m_current_p(0, 0) = m_current_target_link_pose.pose.position.x;
        m_current_p(1, 0) = m_current_target_link_pose.pose.position.y;
        m_current_p(2, 0) = m_current_target_link_pose.pose.position.z;

        getObstacleDistance();

        getJointForce();

        control_msgs::JointJog jog;
        jog.header = m_trajectory.joint_trajectory.header;
        jog.joint_names = m_trajectory.joint_trajectory.joint_names;
        jog.velocities.resize(6);
        for (int i = 0; i < m_link_name.size(); ++i)
        {
            jog.velocities[i] = m_all_force(i, 0);
        }
        m_move->moveJointServo(jog);
        m_apf->iter++;
        if (m_apf->iter > m_apf->max_iter)
        {
            flag = -2;
            m_apf->iter = 0;
            jog.velocities.clear();
            jog.velocities = {0, 0, 0, 0, 0, 0};
            m_move->moveJointServo(jog);
            break;
        }
        m_move_group->setStartStateToCurrentState();
        m_current_target_link_pose = m_move_group->getCurrentPose(m_eef);
        position_err = finishPosition(m_current_target_link_pose, m_target_pose_e);
        orien_err = finishOrien(m_current_target_link_pose, m_target_pose_e);
        clock_t t2 = clock();
        clock_t dur = t2 - t1;
        ROS_INFO_STREAM("iter: " << m_apf->iter << " dur: " << static_cast<double>(dur) / CLOCKS_PER_SEC);
        ROS_INFO_STREAM("position_err: " << position_err << " target_err: " << m_apf->target_err);
        ROS_INFO_STREAM("orien_err: " << orien_err << " target_err_ori: " << m_apf->target_err_ori);
    }
    if (m_is_stop)
    {
        flag = -3;
        m_apf->iter = 0;
    }
    return flag;
}

inline double ApfLocalPlanner::getDistance(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2)
{
    std::vector<double> p1, p2;
    fill(pose1, p1);
    fill(pose2, p2);
    double d = pow((p1[0] - p2[0]), 2) + pow((p1[1] - p2[1]), 2) + pow((p1[2] - p2[2]), 2);
    return sqrt(d);
}

inline double ApfLocalPlanner::getDistance(const geometry_msgs::PoseStamped &pose1, const Eigen::Isometry3d &pose2)
{
    std::vector<double> p1;
    fill(pose1, p1);
    double d = getDistance(pose2, p1);
    return sqrt(d);
}

double ApfLocalPlanner::getDistance(const geometry_msgs::PoseStamped &pose1, const std::vector<double> &pose2)
{
    std::vector<double> p1;
    fill(pose1, p1);
    double d = pow((p1[0] - pose2[0]), 2) + pow((p1[1] - pose2[1]), 2) + pow((p1[2] - pose2[2]), 2);
    return sqrt(d);
}

double ApfLocalPlanner::getDistance(const Eigen::Isometry3d& pose1, const std::vector<double>& pose2)
{
    double d = pow((pose2[0] - pose1(0, 3)), 2) + pow((pose2[1] - pose1(1, 3)), 2) + pow((pose2[2] - pose1(2, 3)), 2);
    return sqrt(d);
}

void ApfLocalPlanner::getObstacleDistance()
{
    getLinkPose();
    m_obs_distance.clear();
    m_obs_distance.resize(m_link_name.size());
    for (size_t i = 0; i < m_link_name.size(); ++i)
    {
        m_obs_distance[i].resize(m_obs_pose.size());
        for (size_t j = 0; j < m_obs_pose.size(); ++j)
        {
            m_obs_distance[i][j] = getDistance(m_link_pose_vec[i], m_obs_pose[j]);
        }
    }
}

inline void ApfLocalPlanner::fill(const geometry_msgs::PoseStamped &pose, std::vector<double> &positin)
{
    positin.resize(3);
    positin[0] = pose.pose.position.x;
    positin[1] = pose.pose.position.y;
    positin[2] = pose.pose.position.z;
}

int ApfLocalPlanner::getJointForce()
{
    m_joint_att_trajectory_p.fill(0);
    m_joint_att_trajectory_w.fill(0);
    m_joint_att_force_w.fill(0);
    m_joint_att_force_p.fill(0);
    m_joint_rep_force.fill(0);
    if (getTrajectoryForce() != 0)
    {
        getTargetPoseForce();
    }
    // getObstacleForce();
    m_all_force = m_joint_att_trajectory_p * m_apf->tra_alfa 
                + m_joint_att_trajectory_w * m_apf->tra_alfa_rot 
                + m_joint_att_force_p * m_apf->target_alfa 
                + m_joint_att_force_w * m_apf->target_alfa_rot 
                + m_joint_rep_force;
}

int ApfLocalPlanner::getTrajectoryForce()
{
    if (getNearbyPoint())
    {
        Eigen::Vector3d euler_err;
        Eigen::Vector3d euler_err1;
        getOrienError(m_current_target_link_pose, (*m_tcp_trajectory_iter), euler_err1);
        euler_err = getOrienError(m_current_target_link_pose, (*m_tcp_trajectory_iter));
        Eigen::Matrix<double, 3, 1> err_p = Eigen::Matrix<double, 3, 1>::Identity();
        Eigen::Matrix<double, 3, 1> err_w = Eigen::Matrix<double, 3, 1>::Identity();
        for (int i = 0; i < 3; ++i)
        {
            err_p(i, 0) = -m_apf->tra_dist_att * m_apf->trajectory_zeta * (m_current_p(i, 0) - (*m_tcp_trajectory_iter)(i, 3));
            err_w(i, 0) = m_apf->tra_dist_att_config * m_apf->trajectory_zeta * euler_err(2 - i);
        }
        Eigen::MatrixXd link_volecity_v = m_volecity_transform.block<6, 3>(0, 0) * err_p;
        Eigen::MatrixXd link_volecity_w = m_volecity_transform.block<6, 3>(0, 3) * err_w;

        // cout << "m_volecity_transform.block<6, 3>(0, 0)" << endl
        //      << m_volecity_transform.block<6, 3>(0, 0) << endl;
        //    cout << "m_volecity_transform.block<6, 3>(0, 3)" << endl
        //      << m_volecity_transform.block<6, 3>(0, 3) << endl;

        m_joint_att_trajectory_p = m_all_jacobian[m_link_name.size() - 2] * link_volecity_v;
        m_joint_att_trajectory_w = m_all_jacobian[m_link_name.size() - 2] * link_volecity_w;
        return 0;
    }
    return -1;
}

bool ApfLocalPlanner::getNearbyPoint()
{
    static int cnt = 0;
    std::list<Eigen::Isometry3d>::iterator iter = m_tcp_trajectory_iter;
    int index = m_trajectory_point_index;
    geometry_msgs::PoseStamped current_pose;
    m_move_group->setStartStateToCurrentState();
    current_pose = m_move_group->getCurrentPose(m_eef);
    int iter_push_index = 2000;

    for (; iter != m_tcp_trajectory.end(); ++iter)
    {
        double dis = getDistance(current_pose, (*iter));
        if (dis > m_apf->global_trajectory_att_inner && dis < m_apf->global_trajectory_att_outer)
        {
            if (m_tcp_trajectory_iter == iter)
            {
                ++cnt;
                // for (size_t i = 0; i < m_obs_pose.size(); i++)
                // {
                //     double dis = getDistance((*iter), m_obs_pose[i]);
                //     if (dis < 0.1)
                //         iter_push_index = 500;
                // }
                if (cnt >= iter_push_index)
                {
                    ++iter;
                    ++index;
                    cnt = 0;
                    ROS_INFO_STREAM("trajectory_point(cnt): " << index);
                }
            }
            setTrajectoryIndex(index, iter);
            return true;
        }
        ++index;
        cnt = 0;
        if (index >= m_tcp_trajectory.size())
        {
            setTrajectoryIndex(index, iter);
            break;
        }
        ROS_INFO_STREAM("trajectory_point(iter): " << index);
    }
    return false;
}

void ApfLocalPlanner::setTrajectoryIndex(int index, std::list<Eigen::Isometry3d>::iterator &iter)
{
    m_tcp_trajectory_iter = iter;
    m_trajectory_point_index = index;
}

int ApfLocalPlanner::getTargetPoseForce()
{

    Eigen::Vector3d euler_err;
    getOrienError(m_current_target_link_pose, m_target_pose_e, euler_err);
    euler_err = getOrienError(m_current_target_link_pose, m_target_pose_e);
    Eigen::Matrix<double, 3, 1> err_p;
    Eigen::Matrix<double, 3, 1> err_w;
    for (int i = 0; i < 3; ++i)
    {
        err_p(i, 0) = -m_apf->dist_att * m_apf->target_zeta * (m_current_p(i, 0) - (m_target_pose_e(i, 3)));
        err_w(i, 0) = m_apf->dist_att_config * m_apf->target_zeta * euler_err(2 - i);
    }

    Eigen::MatrixXd link_volecity_v = m_volecity_transform.block<6, 3>(0, 0) * err_p;
    Eigen::MatrixXd link_volecity_w = m_volecity_transform.block<6, 3>(0, 3) * err_w;
    m_joint_att_force_p = m_all_jacobian[m_link_name.size() - 2] * link_volecity_v;
    m_joint_att_force_w = m_all_jacobian[m_link_name.size() - 2] * link_volecity_w;

    return 0;
}

int ApfLocalPlanner::getObstacleForce()
{
    std::vector<std::vector<Eigen::Matrix<double, 6, 1>>> joint_rep_force;
    std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> obs_rep;
    joint_rep_force.resize(m_link_name.size());
    obs_rep.resize(m_link_name.size());
    // getObstacleDistance();
    for (int i = 0; i < m_link_name.size(); ++i)
    {
        joint_rep_force[i].resize(m_obs_pose.size());
        obs_rep[i].resize(m_obs_pose.size());
        for (size_t j = 0; j < m_obs_pose.size(); ++j)
        {
            joint_rep_force[i][j].fill(0);
            obs_rep[i][j].fill(0);
            if (m_obs_distance[i][j] > m_apf->safety_distance)
            {
                obs_rep[i][j] << 0, 0, 0;
                continue;
            }
            for (int k = 0; k < 3; ++k)
            {
                double dis = m_current_p(k, 0) - m_obs_pose[j][k];
                obs_rep[i][j](k, 0) = m_apf->obs_eta[i] * (1 / m_obs_distance[i][j] - 1 / m_apf->safety_distance) * (1 / (m_obs_distance[i][j] * m_obs_distance[i][j])) * (dis / m_obs_distance[i][j]);
            }
            joint_rep_force[i][j] = m_jacobian_p[i] * obs_rep[i][j];
            joint_rep_force[i][j] *= m_apf->obs_alfa[i];
            m_joint_rep_force += joint_rep_force[i][j];
        }
    }
    return 0;
}

int ApfLocalPlanner::getLinkPose()
{
    m_move_group->setStartStateToCurrentState();
    m_link_pose_vec.clear();
    m_link_pose_vec.resize(m_link_name.size());
    for (size_t i = 0; i < m_link_name.size(); i++)
    {
        m_link_pose_vec[i] = m_move_group->getCurrentPose(m_link_name[i]);
    }
    return 0;
}

void ApfLocalPlanner::getOrienError(const geometry_msgs::PoseStamped &pc, const Eigen::Isometry3d &pt, Eigen::Vector3d &euler_angle)
{
    Eigen::Quaterniond q1c(pc.pose.orientation.w, pc.pose.orientation.x, pc.pose.orientation.y, pc.pose.orientation.z);
    Eigen::Quaterniond q2t(pt.rotation());
    Eigen::Quaterniond e_q = q2t * q1c.inverse();
    euler_angle = e_q.matrix().eulerAngles(2, 1, 0);
}

Eigen::Vector3d ApfLocalPlanner::getOrienError(const geometry_msgs::PoseStamped &p, const Eigen::Isometry3d &p2)
{
    Eigen::Quaterniond q1c(p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z);
    Eigen::Vector3d ec = q1c.matrix().eulerAngles(0, 1, 2);
    Eigen::Vector3d et = p2.rotation().eulerAngles(0, 1, 2);
    return (ec - et);
}

double ApfLocalPlanner::finishPosition(const geometry_msgs::PoseStamped &s, const Eigen::Isometry3d &t)
{
    Eigen::Matrix<double, 3, 1> current_position;
    current_position << s.pose.position.x, s.pose.position.y, s.pose.position.z;
    Eigen::Matrix<double, 3, 1> target_position;
    target_position << t(0, 3), t(1, 3), t(2, 3);
    Eigen::Matrix<double, 3, 1> err = current_position - target_position;
    double err_norm = err.norm();
    return err_norm;
}

double ApfLocalPlanner::finishOrien(const geometry_msgs::PoseStamped &s, const Eigen::Isometry3d &t)
{
    Eigen::Vector3d err;
    // getOrienError(s, t, err);
    err = getOrienError(s, t);
    double err_sum = err.sum();
    return abs(err_sum);
}

void ApfLocalPlanner::JointStateCB(const sensor_msgs::JointStateConstPtr &msg)
{
    m_current_joint_value = msg->position;
}

double ApfLocalPlanner::finishJoint(std::vector<double> current_joint, std::vector<double> target_joint)
{
    double sum = 0;
    for (int i = 0; i < current_joint.size(); i++)
    {
        sum += pow((current_joint[i] - target_joint[i]), 2);
    }
    sum = sqrt(sum);
    return sum;
}

void ApfLocalPlanner::setParam()
{
    m_nh->param<int>("max_iter", m_apf->max_iter, 5000);
    m_nh->param<double>("safety_distance", m_apf->safety_distance, 0.30);

    m_nh->param<double>("target_zeta", m_apf->target_zeta, 0.1);
    m_nh->param<double>("target_alfa", m_apf->target_alfa, 0.5);
    m_nh->param<double>("target_alfa_rot", m_apf->target_alfa_rot, 0.004);
    m_nh->param<double>("dist_att", m_apf->dist_att, 0.01);
    m_nh->param<double>("dist_att_config", m_apf->dist_att_config, 0.01);

    m_nh->param<double>("trajectory_zeta", m_apf->trajectory_zeta, 0.1);
    m_nh->param<double>("tra_alfa", m_apf->tra_alfa, 0.5);
    m_nh->param<double>("tra_alfa_rot", m_apf->tra_alfa_rot, 0.004);
    m_nh->param<double>("tra_dist_att", m_apf->tra_dist_att, 0.01);
    m_nh->param<double>("tra_dist_att_config", m_apf->tra_dist_att_config, 0.01);

    m_nh->param<double>("global_trajectory_att_outer", m_apf->global_trajectory_att_outer, 0.30);
    m_nh->param<double>("global_trajectory_att_inner", m_apf->global_trajectory_att_inner, 0.02);

    m_nh->param<double>("target_err", m_apf->target_err, 0.02);
    m_nh->param<double>("target_err_ori", m_apf->target_err_ori, 0.025);

    vector<double> obs_eta{0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    vector<double> obs_rep{0.03, 0.03, 0.03, 0.03, 0.03, 0.03};
    vector<double> obs_alfa{0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

    m_nh->param<vector<double>>("obs_eta", m_apf->obs_eta, obs_eta);
    m_nh->param<vector<double>>("obs_rep", m_apf->obs_rep, obs_rep);
    m_nh->param<vector<double>>("obs_alfa", m_apf->obs_alfa, obs_alfa);
}

void ApfLocalPlanner::tr2jac(const Eigen::Isometry3d &T)
{
    Eigen::Matrix3d R_inv = T.rotation().inverse();
    Eigen::Matrix3d t;
    t << 0, -T.translation()(2), T.translation()(1),
        T.translation()(2), 0, -T.translation()(0),
        -T.translation()(1), T.translation()(0), 0;
    Eigen::Matrix3d t2 = -R_inv * t;
    Eigen::Matrix<double, 6, 6> Tv;
    Tv << R_inv, t2, Eigen::Matrix3d::Zero(), R_inv;
    m_volecity_transform = Tv;
}

Eigen::Isometry3d ApfLocalPlanner::getTransform(const std::string &target_link, const std::string &source_link)
{
    tf::TransformListener listenter;
    tf::StampedTransform transform;
    listenter.waitForTransform(target_link, source_link, ros::Time(0), ros::Duration(3));
    try
    {
        listenter.lookupTransform(target_link, source_link, ros::Time(0), transform);
    }
    catch(const tf::LookupException& e)
    {
        std::cerr << e.what() << '\n';
        exit(-1);
    }
    Eigen::Isometry3d t;
    tf::Quaternion tf_q = transform.getRotation();
    Eigen::Quaterniond q(tf_q.getW(), tf_q.getX(), tf_q.getY(), tf_q.getZ());
    t.prerotate(q);
    tf::Vector3 ori = transform.getOrigin();
    Eigen::Vector3d trans(ori.getX(), ori.getY(), ori.getZ());
    t.pretranslate(trans);
    return t;
}