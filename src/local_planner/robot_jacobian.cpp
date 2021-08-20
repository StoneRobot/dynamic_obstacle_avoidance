#include "dynamic_obstacle_avoidance/local_planner/robot_jacobian.h"
using namespace std;

RobotJacobian::RobotJacobian(ros::NodeHandle* nh, moveit::planning_interface::MoveGroupInterface *group, const vector<string>& link)
    : m_move_group{group},
    m_nh{nh}
{
    // vector<string> jacobian_link;
    // vector<string> l{"link2", "link3", "link4", "link5", "link6", "pick_link"};
    // m_nh->param<vector<string>>("link", jacobian_link, l);
    // m_joint_link = m_move_group->getLinkNames();
    m_joint_link = link;
    current_state = m_move_group->getCurrentState();
    model = current_state->getRobotModel();
    j_model_group = current_state->getJointModelGroup(m_move_group->getName());
}

RobotJacobian::~RobotJacobian()
{
}

bool RobotJacobian::getJacobian(Eigen::MatrixXd &jacobian, const std::string &frame)
{
    Eigen::Vector3d a(0, 0, 0);
    const robot_model::LinkModel *lm = model->getLinkModel(frame);

    // getJacobian(const JointModelGroup* group,                 const LinkModel* link, 
    //const Eigen::Vector3d& reference_point_position,
    //Eigen::MatrixXd& jacobian,                        bool use_quaternion_representation = false)
    return current_state->getJacobian(j_model_group, lm, a, jacobian);
}

bool RobotJacobian::getAllJacobian(std::vector<Eigen::MatrixXd> &jacobians)
{
    std::vector<std::string> joint_name;
    jacobians.clear();
    jacobians.resize(m_joint_link.size());
    int n = 0;
    vector<double> j_val = m_move_group->getCurrentJointValues();
    current_state->setJointGroupPositions(j_model_group, j_val);
    for (auto i : m_joint_link)
    {
        // clock_t t = clock();
        Eigen::MatrixXd jacobian;
        getJacobian(jacobian, i);                   
                      // jacobians[n] = jacobian.transpose();               
        jacobians[n] =  jacobian.completeOrthogonalDecomposition().pseudoInverse();       //雅可比矩阵   求伪逆

        //cout <<( jacobian.inverse()==jacobians[n] ? " true" : "false" ) << endl;
        //auto inv = jacobian.inverse();
              //auto  Ainv =  jacobian.completeOrthogonalDecomposition().pseudoInverse();
              //auto wei_ni = pseudoInverse(jacobian);   
        //cout <<n<<":     "<< jacobians[n] << endl;
        // cout << "get one jacobian: " << clock() - t << endl;
             // cout << Ainv << endl << jacobians[n] << endl;
        ++n;
    }
    return true;
}
