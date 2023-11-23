#include "advanced_robotics_franka_controllers/robot.h"

namespace NJSDF
{
    RobotModel::RobotModel()
    {
        q_rbdl_.resize(dof);
        q_rbdl_.setZero();
        qdot_rbdl_.resize(dof);
        qdot_rbdl_.setZero();
        
        m_pos_.setZero();
        m_Ori_.setZero();
        m_j_rbdl_.setZero(); 
        m_j_.setZero();

        setRobot();
    }

    RobotModel::~RobotModel()
    {

    }

    void RobotModel::setRobot()
    {
        model_ = std::make_shared<RigidBodyDynamics::Model>();

        std::string file_path = "/home/dyros/jh_ws/src/advanced_robotics_franka_controllers/urdf/panda.urdf";
        RigidBodyDynamics::Addons::URDFReadFromFile(file_path.c_str(), model_.get(), false);

        std::string link_name = "panda_";
        for(size_t i=0; i<num_links-1; ++i) body_id_[i] = model_->GetBodyId((link_name + "link" + std::to_string(i)).c_str());
        body_id_[num_links-1] = model_->GetBodyId((link_name + "hand").c_str());

        for(size_t i=0; i<num_links; ++i) com_position_[i] = Eigen::Vector3d(0, 0, 0);
    }

    void RobotModel::RobotModel::Jacobian(const int & frame_id) 
    {
        Eigen::MatrixXd j_temp;
        j_temp.setZero(ee_dof, dof);

        RigidBodyDynamics::CalcPointJacobian6D(*model_, q_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1], j_temp, true);
        RigidBodyDynamics::CalcPointJacobian6D(*model_, q_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1], j_temp, true);
        
        for (size_t i = 0; i<2; i++)  m_j_.block<3, dof>(i * 3, 0) = j_temp.block<3, dof>(3 - i * 3, 0);

    }
    
    void RobotModel::RobotModel::Position(const int & frame_id) 
    { 
        m_pos_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(*model_, q_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1], true);
    }

    void RobotModel::RobotModel::Orientation(const int & frame_id) 
    {
        m_Ori_ = RigidBodyDynamics::CalcBodyWorldOrientation(*model_, q_rbdl_, body_id_[frame_id - 1], true).transpose();
    }
    
    void RobotModel::RobotModel::Transformation(const int & frame_id) 
    {
        Position(frame_id);
        Orientation(frame_id);
        m_Trans_.linear() = m_Ori_;
        m_Trans_.translation() = m_pos_;
    }

    void RobotModel::RobotModel::getUpdateKinematics(const Eigen::VectorXd & q, const Eigen::VectorXd & qdot) 
    {
        q_rbdl_ = q;
        qdot_rbdl_ = qdot;	
        RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q_rbdl_, &qdot_rbdl_, NULL);
    }
}