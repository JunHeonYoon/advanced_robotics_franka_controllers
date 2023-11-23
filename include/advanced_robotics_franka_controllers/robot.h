#ifndef NJSDF_ROBOT_MODEL_H
#define NJSDF_ROBOT_MODEL_H

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "advanced_robotics_franka_controllers/common.h"

namespace NJSDF
{
    class RobotModel

    {
        public:
            // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            RobotModel();
            ~RobotModel();

            void getUpdateKinematics(const Eigen::VectorXd & q, const Eigen::VectorXd & qdot);
            const Eigen::Matrix<double, ee_dof, dof> & getJacobian(const int & frame_id) 
            {
				Jacobian(frame_id);
				return m_j_;
			}
            const Eigen::Vector3d & getPosition(const int & frame_id) 
            {
				Position(frame_id);
				return m_pos_;
			}
			const Eigen::Matrix3d & getOrientation(const int & frame_id) 
            {
				Orientation(frame_id);
				return m_Ori_;
			}
            const Eigen::Affine3d & getTransformation(const int & frame_id) 
            {
				Transformation(frame_id);
				return m_Trans_;
			}
            const Eigen::VectorXd & getJointPosition() 
            {
				return q_rbdl_;
            }

			Eigen::VectorXd q_rbdl_;
			Eigen::VectorXd qdot_rbdl_;
			
			Eigen::Matrix<double, 3, 1> m_pos_;
			Eigen::Matrix<double, 3, 3>  m_Ori_;
			Eigen::Matrix<double, ee_dof, dof> m_j_rbdl_, m_j_;

			Eigen::Affine3d m_Trans_;



        // private:
            void Jacobian(const int & frame_id);
			void Position(const int & frame_id);
			void Orientation(const int & frame_id);
			void Transformation(const int & frame_id);
			void setRobot();

			std::shared_ptr<RigidBodyDynamics::Model> model_;
            unsigned int body_id_[num_links];  
			Eigen::Matrix<double, 3, 1> com_position_[num_links];

    };
}


#endif // NJSDF_ROBOT_MODEL_H