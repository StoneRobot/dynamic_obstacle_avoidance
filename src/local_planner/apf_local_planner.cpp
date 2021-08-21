#include "dynamic_obstacle_avoidance/local_planner/apf_local_planner.h"
#include <math.h>
#include <cmath>
#include <algorithm>
#include <time.h>
#include <tf/transform_listener.h>
#include <random>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>

#define CLAMP(x, min, max) ((x) > (max) ? (max) : ((x) < (min) ? (min) : x))
using namespace std;

int num = 0;
double Array[1000][3];

ApfLocalPlanner::ApfLocalPlanner(ros::NodeHandle *nh, moveit::planning_interface::MoveGroupInterface *group)
    : ILocalPlanner{nh, group},
      m_is_stop{false}
{
    vector<string> l{"link2", "link3", "link4", "link5", "link6", "pick_link"};
    m_nh->param<vector<string>>("link", m_link_name, l);
    m_jacobian_p.resize(m_link_name.size());
    m_robot_jacobian = new RobotJacobian(nh, m_move_group, m_link_name);
    m_apf = new ApfParam; // ApfParam     势场参数结构体

    //订阅
    m_joint_state_sub = m_nh->subscribe("/joint_states", 10, &ApfLocalPlanner::JointStateCB, this);

    setParam(); //设置参数

    //获取4x4的转换矩阵
    //来自Robotics System Toolbox工具箱
    //m_link_name[m_link_name.size() - 2]=m_link_name[6-2]=m_link_name[4]，即“link6”
    //同理可得，后面的是“pick_link”
    //pick_link 转 link6
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
    m_apf->iter = 0;              //记录迭代次数
    m_trajectory_point_index = 1; //全局轨迹的点的迭代器，索引到第二个点

    //c.begin();           返回指向容器 最开始位置数据的指针
    m_tcp_trajectory_iter = m_tcp_trajectory.begin(); //std::list<Eigen::Isometry3d> m_tcp_trajectory;
    ++m_tcp_trajectory_iter;
    int flag = 0;

    flag = executeGlobalTra(); //判断flag   全局轨迹执行器
    if (flag == -1)            //小于安全距离，执行人工势场法
        flag = executeApf();   //局部规划器
    return flag;
}

int ApfLocalPlanner::executeGlobalTra()
{
    while (ros::ok() && !m_is_stop)
    {
        static int cnt = 0;    //计量有障碍物在附近的点的逼近次数
        getObstacleDistance(); //得到m_obs_distance[i][j]的值
        for (size_t i = 0; i < m_obs_distance.size(); i++)
            for (size_t j = 0; j < m_obs_distance[i].size(); j++)
                if (m_obs_distance[i][j] < m_apf->safety_distance) //小于安全距离
                    return -1;

        //全局规划器，  执行
        control_msgs::JointJog jog; //第一个jog
        jog.header.frame_id = m_trajectory.joint_trajectory.header.frame_id;
        jog.joint_names = m_trajectory.joint_trajectory.joint_names;

        int index; //表示moveit_servo需要到达的点     指下一个需要执行的点的索引

        //①   _index代表索引， m_trajectory.joint_trajectory.points.size()代表有多少个点
        m_trajectory_point_index == m_trajectory.joint_trajectory.points.size() - 1 ? index = m_trajectory_point_index - 1 : index = m_trajectory_point_index;
        // 避免到检索到最后一个点              trajectory已经规划好的

        //   jog.velocities[i] 第i+1个关节的速度
        jog.velocities.resize(6);
        for (int i = 0; i < 6; i++)
        {
            //从①中选择合适的点，在②中获取轨迹的速度，然后在下面的moveJointServo执行
            jog.velocities[i] = m_trajectory.joint_trajectory.points[index].velocities[i] * 1.5; //②                 改     0.3
        }

        //两点之间的持续时间
        jog.duration = m_trajectory.joint_trajectory.points[m_trajectory_point_index].time_from_start.toSec() -
                       m_trajectory.joint_trajectory.points[m_trajectory_point_index - 1].time_from_start.toSec();
        m_move->moveJointServo(jog);

        //关节角总误差，误差小于设定值才走下一个点
        double err = finishJoint(m_current_joint_value, m_trajectory.joint_trajectory.points[m_trajectory_point_index].positions);
        //ROS_INFO_STREAM("err: " << err);
        if (err < 0.052333333)
        {
            //m_trajectory_point_index++;
            ++m_trajectory_point_index; //索引
            ROS_INFO_STREAM("m_trajectory_point_index: " << m_trajectory_point_index);
            ++m_tcp_trajectory_iter; //容器
        }
        // ++cnt;
        //到达最后一点
        if (m_trajectory_point_index >= m_trajectory.joint_trajectory.points.size())
            break;
    }
    return 0;
}

int ApfLocalPlanner::executeApf()
{
    int flag = 0;
    getLinkPose();
    m_current_target_link_pose = m_move_group->getCurrentPose(m_eef); //m_eef表示pick_link

    //位置误差
    double position_err = finishPosition(m_current_target_link_pose, m_target_pose_e);

    //姿态误差
    double orien_err = finishOrien(m_current_target_link_pose, m_target_pose_e);

    //位置、姿态误差大于设定误差时
    while (ros::ok() && !m_is_stop && (position_err > m_apf->target_err || orien_err > m_apf->target_err_ori))
    {
        clock_t t1 = clock();

        m_robot_jacobian->getAllJacobian(m_all_jacobian);
        vector<Eigen::MatrixXd> jacobian_p;
        jacobian_p.resize(m_link_name.size());

        for (int i = 0; i < jacobian_p.size(); ++i)
        {
            //m_jacobian_p为vector， 取第i个雅可比矩阵的前三列赋值给m_jacobian_p的第i个vector      ？
            m_jacobian_p[i] = m_all_jacobian[i].block<6, 3>(0, 0);
            //m_jacobian_p为vector        每个i为6x3矩阵
        }

        //link6       //这一块没用到
        m_target_jacobian_w = m_all_jacobian[m_link_name.size() - 2].block<6, 3>(0, 3);
        m_target_jacobian_p = m_all_jacobian[m_link_name.size() - 2].block<6, 3>(0, 0);

        getLinkPose();
        m_current_target_link_pose = m_move_group->getCurrentPose(m_eef);
        m_current_p(0, 0) = m_current_target_link_pose.pose.position.x;
        m_current_p(1, 0) = m_current_target_link_pose.pose.position.y;
        m_current_p(2, 0) = m_current_target_link_pose.pose.position.z;

        getObstacleDistance();

        getJointForce();

        //局部规划的执行器
        control_msgs::JointJog jog;
        jog.header = m_trajectory.joint_trajectory.header;
        jog.joint_names = m_trajectory.joint_trajectory.joint_names;
        jog.velocities.resize(6);
        for (int i = 0; i < m_link_name.size(); ++i)
        {
            jog.velocities[i] = m_all_force(i, 0); //force即velocity      改
        }
        m_move->moveJointServo(jog);
        m_apf->iter++;
        if (m_apf->iter > m_apf->max_iter) //超过最大迭代次数
        {
            flag = -2;
            m_apf->iter = 0;
            jog.velocities.clear();
            jog.velocities = {0, 0, 0, 0, 0, 0};
            m_move->moveJointServo(jog); //停止运动
            break;
        }
        m_move_group->setStartStateToCurrentState();
        m_current_target_link_pose = m_move_group->getCurrentPose(m_eef);

        //位置误差
        position_err = finishPosition(m_current_target_link_pose, m_target_pose_e);

        //姿态误差
        orien_err = finishOrien(m_current_target_link_pose, m_target_pose_e);

        clock_t t2 = clock();
        clock_t dur = t2 - t1;
        //迭代时间、误差显示
        ROS_INFO_STREAM("iter: " << m_apf->iter << " dur: " << static_cast<double>(dur) / CLOCKS_PER_SEC);
        ROS_INFO_STREAM("position_err: " << position_err << " target_err: " << m_apf->target_err);
        ROS_INFO_STREAM("orien_err: " << orien_err << " target_err_ori: " << m_apf->target_err_ori);
    }

    //异常，非自然停止
    if (m_is_stop) //m_is_stop
    {
        flag = -3;
        m_apf->iter = 0;
    }
    return flag; //？
}

//四个getDistance函数
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

double ApfLocalPlanner::getDistance(const Eigen::Isometry3d &pose1, const std::vector<double> &pose2)
{
    double d = pow((pose2[0] - pose1(0, 3)), 2) + pow((pose2[1] - pose1(1, 3)), 2) + pow((pose2[2] - pose1(2, 3)), 2);
    return sqrt(d);
}

//计算距离
//m_link_pose_vec[i]表示link的原点、m_obs_pose[j]表示第几个最近点
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

//把geometry_msgs::PoseStamped转换成vector<double>
inline void ApfLocalPlanner::fill(const geometry_msgs::PoseStamped &pose, std::vector<double> &positin)
{
    positin.resize(3);
    positin[0] = pose.pose.position.x;
    positin[1] = pose.pose.position.y;
    positin[2] = pose.pose.position.z;
}

int ApfLocalPlanner::getJointForce()
{
    m_joint_att_trajectory_p.fill(0); //轨迹跟随 / 目标跟随
    m_joint_att_trajectory_w.fill(0);
    m_joint_att_force_w.fill(0);
    m_joint_att_force_p.fill(0);
    m_joint_rep_force.fill(0);
    m_all_force.fill(0);

    //局部轨迹跟随器
    if (getTrajectoryForce() != 0)
    {
        //获取目标跟随器的力
        getTargetPoseForce();
    }

    //获取障碍物排斥速度                 斥力
    getObstacleForce();

    //_alfa 对由末端位置速度转换得到的关节得到数值乘以的比例
    //_alfa_rot对由末端姿态速度转换得到的关节得到数值乘以的比例
    //合成
    Eigen::Matrix<double, 6, 1> att = Eigen::Matrix<double, 6, 1>::Zero();
    att = m_joint_att_trajectory_p * m_apf->tra_alfa + m_joint_att_force_p * m_apf->target_alfa;
    Eigen::Matrix<double, 6, 1> rep = Eigen::Matrix<double, 6, 1>::Zero();
    rep = m_joint_rep_force;
    m_all_force = att + rep;

    // m_all_force = m_joint_att_trajectory_p * m_apf->tra_alfa                                                  //tra_alfa=1.2     0.6
    //               + m_joint_att_trajectory_w * m_apf->tra_alfa_rot + m_joint_att_force_p * m_apf->target_alfa //target_alfa=10   5
    //               + m_joint_att_force_w * m_apf->target_alfa_rot + m_joint_rep_force;
}

//获取局部轨迹跟随器的力
int ApfLocalPlanner::getTrajectoryForce()
{
    int ret = -1;
    //从全局轨迹中选取符合跟随器的点，如果符合返回true，如果机器人震荡，跟这个函数有关
    if (getNearbyPoint()) //哪个最近点
    {
        double position_err;
        //位置误差
        position_err = finishPosition(m_current_target_link_pose, (*m_tcp_trajectory_iter));
        //计算位置和姿态误差
        //人工势场法
        // 位置
        Eigen::Matrix<double, 3, 1> err_p = Eigen::Matrix<double, 3, 1>::Identity();
        for (int i = 0; i < 3; ++i)
        {
            //后面乘以误差的倒数是为了自适应，建议调整公式
            err_p(i, 0) = -m_apf->tra_dist_att * m_apf->trajectory_zeta *
                          (m_current_p(i, 0) - (*m_tcp_trajectory_iter)(i, 3)) * (1 / abs(position_err)); //1.2*1*(  -  )* (1/())
        }
        // 姿态
        auto p_ = m_current_target_link_pose;
        Eigen::Quaterniond q0(p_.pose.orientation.w, p_.pose.orientation.x, p_.pose.orientation.y, p_.pose.orientation.z);
        Eigen::Quaterniond q1((*m_tcp_trajectory_iter).rotation());

        Eigen::Quaterniond q_next = q0.slerp(0.5, q1);

        p_.pose.position.x += err_p(0, 0) * 0.01;
        p_.pose.position.y += err_p(1, 0) * 0.01;
        p_.pose.position.z += err_p(2, 0) * 0.01;

        p_.pose.orientation.w = q_next.w();
        p_.pose.orientation.x = q_next.x();
        p_.pose.orientation.y = q_next.y();
        p_.pose.orientation.z = q_next.z();
        int cnt = 0;
        do
        {
            vector<double> joint;
            if (!IK(p_, joint))
                break;
            if (finishJoint(m_current_joint_value, joint) < 1.57)
            {
                for (size_t i = 0; i < 6; i++)
                {
                    m_joint_att_trajectory_p(i, 0) = joint[i] - m_current_joint_value[i];
                }
                ret = 0;
                break;
            }
            cnt++;
        } while (ros::ok() && cnt < 8);
    }
    return ret; //规划失败
}

bool ApfLocalPlanner::getNearbyPoint() //判断最近点
{
    static int cnt = 0; //计量有障碍物在附近的点的逼近次数
    bool flag = false;
    std::list<Eigen::Isometry3d>::iterator iter = m_tcp_trajectory_iter; //？
    int index = m_trajectory_point_index;                                //？
    geometry_msgs::PoseStamped current_pose;
    m_move_group->setStartStateToCurrentState();
    current_pose = m_move_group->getCurrentPose(m_eef);
    int iter_push_index = 2000;

    //选取轨迹点的方法
    //选择环中的点，如果环太小，速度太大，会造成震荡
    //如果附近有障碍物，则减少对这个点的迭代次数
    //判断不是末位
    for (; iter != m_tcp_trajectory.end(); ++iter)
    {
        double dis = getDistance(current_pose, (*iter)); // Eigen::Isometry3d

        //距离在环内
        if (dis > m_apf->global_trajectory_att_inner && dis < m_apf->global_trajectory_att_outer)
        {
            if (m_tcp_trajectory_iter == iter) //？
            {
                ++cnt;
                for (size_t i = 0; i < m_obs_pose.size(); i++)
                {
                    double dis_obs = getDistance((*iter), m_obs_pose[i]); //
                    if (dis_obs < 0.05)
                    {
                        iter_push_index = 300; //300？
                        break;                 //跳出for循环？
                    }
                }
                if (cnt >= iter_push_index)
                {
                    ++iter;
                    ++index;
                    cnt = 0;
                    if (index >= m_tcp_trajectory.size())
                    {
                        setTrajectoryIndex(index, iter);
                        flag = false;
                        break; //
                    }
                    ROS_INFO_STREAM("trajectory_point(cnt): " << index);
                }
                setTrajectoryIndex(index, iter);
                flag = true;
                break; //
            }
            setTrajectoryIndex(index, iter);
            cnt = 0;
            flag = true;
            break;
        }
        ++index;
        cnt = 0;
        if (index >= m_tcp_trajectory.size())
        {
            ++iter;
            setTrajectoryIndex(index, iter);
            flag = false;
            break;
        }
        ROS_INFO_STREAM("trajectory_point(iter): " << index);
    }
    return flag;
}

//设置轨迹索引
void ApfLocalPlanner::setTrajectoryIndex(int index, std::list<Eigen::Isometry3d>::iterator &iter)
{
    m_tcp_trajectory_iter = iter;
    m_trajectory_point_index = index;
}

//目标跟随器的力
int ApfLocalPlanner::getTargetPoseForce()
{
    int ret = -1;
    // getOrienError(m_current_target_link_pose, m_target_pose_e, euler_err); //
    double position_err;
    position_err = finishPosition(m_current_target_link_pose, (*m_tcp_trajectory_iter));

    Eigen::Matrix<double, 3, 1> err_p;
    //人工势场法
    for (int i = 0; i < 3; ++i) //   for (int i = 2; i >-1;--i)     //
    {
        err_p(i, 0) = -m_apf->dist_att * m_apf->target_zeta *
                      (m_current_p(i, 0) - m_target_pose_e(i, 3)) * (1 / abs(position_err));
    }

    // 姿态
    auto p_ = m_current_target_link_pose;
    Eigen::Quaterniond q0(p_.pose.orientation.w, p_.pose.orientation.x, p_.pose.orientation.y, p_.pose.orientation.z);
    Eigen::Quaterniond q1(m_target_pose_e.rotation());
    double err = q0.angularDistance(q1);
    double t = err > 1 ? 0.1 : (1 - err);
    Eigen::Quaterniond q_next = q0.slerp(t, q1);

    p_.pose.position.x += err_p(0, 0) * 0.01;
    p_.pose.position.y += err_p(1, 0) * 0.01;
    p_.pose.position.z += err_p(2, 0) * 0.01;

    p_.pose.orientation.w = q_next.w();
    p_.pose.orientation.x = q_next.x();
    p_.pose.orientation.y = q_next.y();
    p_.pose.orientation.z = q_next.z();
    int cnt = 0;
    do
    {
        vector<double> joint;
        if (!IK(p_, joint))
            break;
        if (finishJoint(m_current_joint_value, joint) < 1.57)
        {
            for (size_t i = 0; i < 6; i++)
            {
                m_joint_att_force_p(i, 0) = joint[i] - m_current_joint_value[i];
            }
            ret = 0;
            break;
        }
        cnt++;
    } while (ros::ok() && cnt < 8);
    return ret;
}

//获取排斥速度
int ApfLocalPlanner::getObstacleForce()
{
    std::vector<std::vector<Eigen::Matrix<double, 6, 1>>> joint_rep_force;
    std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> obs_rep;
    joint_rep_force.resize(m_link_name.size());
    obs_rep.resize(m_link_name.size());

    //getObstacleDistance();

    for (int i = 0; i < m_link_name.size(); ++i) //遍历link
    {
        joint_rep_force[i].resize(m_obs_pose.size());
        obs_rep[i].resize(m_obs_pose.size());
        for (size_t j = 0; j < m_obs_pose.size(); ++j) //遍历障碍物点
        {
            joint_rep_force[i][j].fill(0);
            obs_rep[i][j].fill(0);

            //距离大于安全距离，斥力为零
            //先给障碍物斥力赋值，再传到关节上
            if (m_obs_distance[i][j] > m_apf->safety_distance)
            {
                obs_rep[i][j] << 0, 0, 0;
                continue; //跳过j这个点
            }
            for (int k = 0; k < 3; ++k)
            {
                double dis = m_current_p(k, 0) - m_obs_pose[j][k]; //第j个vector  第k个元素
                obs_rep[i][j](k, 0) = m_apf->obs_eta[i] * (1 / m_obs_distance[i][j] - 1 / m_apf->safety_distance) *
                                      (1 / (m_obs_distance[i][j] * m_obs_distance[i][j])) * (dis / m_obs_distance[i][j]);
                obs_rep[i][j](k, 0) = obs_rep[i][j](k, 0) * getRandon(1.3, 1.8); //防止陷入局部极小值      改    (1.3, 1.8)
            }
            joint_rep_force[i][j] = m_jacobian_p[i] * obs_rep[i][j];
            //笛卡尔空间转关节空间    [6x3]  [3x1] =[6x1]

            joint_rep_force[i][j] *= m_apf->obs_alfa[i];

            //关节的斥力和   排斥速度
            m_joint_rep_force += joint_rep_force[i][j]; //在关节空间斥力相加
        }
    }
    return 0;
}

//m_link_pose_vec[i]       获取6个link的原点坐标
int ApfLocalPlanner::getLinkPose()
{
    m_move_group->setStartStateToCurrentState();
    m_link_pose_vec.clear();
    m_link_pose_vec.resize(m_link_name.size());
    for (size_t i = 0; i < m_link_name.size(); i++)
    {
        m_link_pose_vec[i] = m_move_group->getCurrentPose(m_link_name[i]); //
    }
    return 0;
}

//两个 getOrienError   &回传
void ApfLocalPlanner::getOrienError(const geometry_msgs::PoseStamped &pc, const Eigen::Isometry3d &pt, Eigen::Vector3d &euler_angle)
{
}

Eigen::Vector3d ApfLocalPlanner::getOrienError(const geometry_msgs::PoseStamped &p, const Eigen::Isometry3d &p2)
{
    Eigen::Quaterniond q1c(p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z);
    Eigen::Vector3d ec = q1c.matrix().eulerAngles(0, 1, 2);  //四元数转欧拉角   012表示 RPY 顺序
    Eigen::Vector3d et = p2.rotation().eulerAngles(0, 1, 2); //旋转矩阵转欧拉角
    m_move_group->setStartStateToCurrentState();
    std::vector<double> rpy = m_move_group->getCurrentRPY(m_eef); //？？
    Eigen::Vector3d err = ec - et;                                //当前-目标
    return err;
}

double ApfLocalPlanner::finishPosition(const geometry_msgs::PoseStamped &s, const Eigen::Isometry3d &t)
{
    Eigen::Matrix<double, 3, 1> current_position;
    current_position << s.pose.position.x, s.pose.position.y, s.pose.position.z;

    cout << "当前位置点：" << s.pose.position.x << "  " << s.pose.position.y << "  " << s.pose.position.z << endl;

    Array[num][0] = s.pose.position.x;
    Array[num][1] = s.pose.position.y;
    Array[num][2] = s.pose.position.z;
    num++;

    ofstream ofile;
    ofile.open("/home/fshs/catkin_ws/avoid_first.txt", ios::out | ios::trunc);

    for (int i = 0; i < num; i++) // 写入数据
    {
        ofile << Array[i][0] << "," << Array[i][1] << "," << Array[i][2] << endl;
        //ofile<<std::endl;
        //fprintf(fp, "%f,%f,%f\n",  Array[i][0], Array[i][1], Array[i][2]);
    }
    ofile.close();

    Eigen::Matrix<double, 3, 1> target_position;
    target_position << t(0, 3), t(1, 3), t(2, 3);
    cout << "目标位置点：" << t(0, 3) << "  " << t(1, 3) << "  " << t(2, 3) << endl;
    Eigen::Matrix<double, 3, 1> err = current_position - target_position;
    double err_norm = err.norm(); //二范数   元素模的平方和开根号
    return err_norm;
}

double ApfLocalPlanner::finishOrien(const geometry_msgs::PoseStamped &s, const Eigen::Isometry3d &t)
{
    Eigen::Quaterniond q1c(s.pose.orientation.w, s.pose.orientation.x, s.pose.orientation.y, s.pose.orientation.z);
    Eigen::Quaterniond q2t(t.rotation());         //使用旋转矩阵来对四元數进行初始化
    Eigen::Quaterniond e_q = q2t * q1c.inverse(); //求逆
    double e = q1c.angularDistance(q2t);
    return e;
}

void ApfLocalPlanner::JointStateCB(const sensor_msgs::JointStateConstPtr &msg)
{
    m_current_joint_value = msg->position;
}

//关节角总误差
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
    m_nh->param<int>("max_iter", m_apf->max_iter, 10000);
    m_nh->param<double>("safety_distance", m_apf->safety_distance, 0.3);

    m_nh->param<double>("target_zeta", m_apf->target_zeta, 0.1);           //1
    m_nh->param<double>("target_alfa", m_apf->target_alfa, 0.5);           //10
    m_nh->param<double>("target_alfa_rot", m_apf->target_alfa_rot, 0.004); //0.5
    m_nh->param<double>("dist_att", m_apf->dist_att, 0.01);                //10
    m_nh->param<double>("dist_att_config", m_apf->dist_att_config, 0.01);  //0.5

    m_nh->param<double>("trajectory_zeta", m_apf->trajectory_zeta, 0.1);          //1
    m_nh->param<double>("tra_alfa", m_apf->tra_alfa, 0.5);                        //1.2
    m_nh->param<double>("tra_alfa_rot", m_apf->tra_alfa_rot, 0.004);              //0.5
    m_nh->param<double>("tra_dist_att", m_apf->tra_dist_att, 0.01);               //1.2
    m_nh->param<double>("tra_dist_att_config", m_apf->tra_dist_att_config, 0.01); //0.5

    m_nh->param<double>("global_trajectory_att_outer", m_apf->global_trajectory_att_outer, 0.30);
    m_nh->param<double>("global_trajectory_att_inner", m_apf->global_trajectory_att_inner, 0.08);

    m_nh->param<double>("target_err", m_apf->target_err, 0.02); //0.02
    m_nh->param<double>("target_err_ori", m_apf->target_err_ori, 0.0174444444);

    vector<double> obs_eta{0.1, 0.1, 0.1, 0.1, 0.1, 0.1}; //0.5
    // vector<double> obs_rep{0.03, 0.03, 0.03, 0.03, 0.03, 0.03};
    vector<double> obs_alfa{0.5, 0.5, 0.5, 0.5, 0.5, 0.5}; //0.01

    m_nh->param<vector<double>>("obs_eta", m_apf->obs_eta, obs_eta);
    // m_nh->param<vector<double>>("obs_rep", m_apf->obs_rep, obs_rep);
    m_nh->param<vector<double>>("obs_alfa", m_apf->obs_alfa, obs_alfa);
}

//构造速度变换矩阵     对什么对象做转换取决于T的参数           书本P111
//link6的速度转至pick_link
void ApfLocalPlanner::tr2jac(const Eigen::Isometry3d &T)
{
    Eigen::Matrix3d R_inv = T.rotation().inverse(); //T的旋转矩阵部分求逆
    Eigen::Matrix3d t;
    t << 0, -T.translation()(2), T.translation()(1),
        T.translation()(2), 0, -T.translation()(0),
        -T.translation()(1), T.translation()(0), 0; // T.translation()表示T的平移矩阵   t=[0, -pz, py, pz, 0, px, -py, px, 0]
    Eigen::Matrix3d t2 = -R_inv * t;
    Eigen::Matrix<double, 6, 6> Tv; //6x6速度变换矩阵
    Tv << R_inv, t2, Eigen::Matrix3d::Zero(), R_inv;
    m_volecity_transform = Tv; // 速度变换矩阵（6x6）
}

Eigen::Isometry3d ApfLocalPlanner::getTransform(const std::string &target_link, const std::string &source_link)
{
    tf::TransformListener listenter; //定义监听器；

    //tf::StampedTransform是ros里代表TF变换的数据类型,一般在机器人上
    //读取雷达坐标系到机器人坐标系的信息就是通过读取tf信息.
    tf::StampedTransform transform; //定义存放变换关系的变量

    //waitForTransform() 函数来等待 tf 的坐标转换线程得到你想要的时间点的坐标转换数据。
    //简单的说：waitForTransform() 就是一个安全程序。
    //监听两个坐标系之间的变换
    listenter.waitForTransform(target_link, source_link, ros::Time(0), ros::Duration(3)); //？
    try
    {
        listenter.lookupTransform(target_link, source_link, ros::Time(0), transform);
    }
    catch (const tf::LookupException &e)
    {
        std::cerr << e.what() << '\n';
        exit(-1);
    }

    Eigen::Isometry3d t;
    tf::Quaternion tf_q = transform.getRotation();

    //Eigen中四元数Quaterniond的初始的方法
    //Eigen::Quaterniond q1(w, x, y, z);// 第一种方式，这里用的是这种
    //Eigen::Quaterniond q2(Vector4d(x, y, z, w));// 第二种方式
    //Eigen::Quaterniond q2(Matrix3d(R));// 第三种方式
    Eigen::Quaterniond q(tf_q.getW(), tf_q.getX(), tf_q.getY(), tf_q.getZ());
    t.prerotate(q);                          //四元数转旋转矩阵
    tf::Vector3 ori = transform.getOrigin(); //transform.getOrigin().x()
    Eigen::Vector3d trans(ori.getX(), ori.getY(), ori.getZ());
    t.pretranslate(trans);
    return t;
}

//获取随机数
double ApfLocalPlanner::getRandon(double a, double b)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(a, b);
    double i = dis(gen);
    // ROS_INFO_STREAM("random" << i);
    return i;
}

bool ApfLocalPlanner::IK(const geometry_msgs::PoseStamped &p, std::vector<double> &j)
{
    bool ret = false;
    moveit::core::RobotStatePtr robot_state = m_move_group->getCurrentState();
    const robot_model::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(m_move_group->getName());
    int attempts = 10;
    double timeout = 0.5;
    int cnt = 0;
    while (ros::ok() && cnt < 5)
    {
        if (robot_state->setFromIK(joint_model_group, p.pose, m_eef, attempts, timeout))
        {
            ROS_INFO_STREAM("IK succeed");
            ret = true;
            robot_state->copyJointGroupPositions(joint_model_group, j);
            break;
        }
        cnt++;
        ROS_INFO_STREAM("IK cnt: " << cnt);
    }
    return ret;
}
