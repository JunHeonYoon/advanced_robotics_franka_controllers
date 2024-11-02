#ifndef QP_JOINT_POSITION_H
#define QP_JOINT_POSITION_H

#include "advanced_robotics_franka_controllers/QP_base.h"
#include <nlohmann/json.hpp>
#include "math_type_define.h"

using json = nlohmann::json;

namespace QP
{
    class JointPosition : public Base
    {
        public:
            JointPosition();
            void setCurrentState(const Eigen::Matrix<double, 7, 1> &q_current, const Eigen::Matrix<double, 7, 1> &qdot_current);
            void setDesiredJointPosition(const Eigen::Matrix<double, 7, 1> &q_desired);
            bool getOptJointVel(Eigen::Matrix<double, 7, 1> &opt_qdot, TimeDuration &time_status);

        private:
            const int nq_ = 7; // 7 for panda dof
            struct QPIndex
            {
                // q dot
                int dq1 = 0;
                int dq2 = 1;
                int dq3 = 2;
                int dq4 = 3;
                int dq5 = 4;
                int dq6 = 5;
                int dq7 = 6;

                // q error
                int qe1 = 7;
                int qe2 = 8;
                int qe3 = 9;
                int qe4 = 10;
                int qe5 = 11;
                int qe6 = 12;
                int qe7 = 13;

                int con_error = 0;  // 0-6
                int con_q = 7;      // 7-13
                int con_qdot = 14;  // 14-20
                int con_qddot = 21; // 21-27
            }si_index_;

            Eigen::Matrix<double, 7, 1> q_current_;
            Eigen::Matrix<double, 7, 1> qdot_current_;
            Eigen::Matrix<double, 7, 1> q_desired_;

            const double hz_ = 200.;

            Eigen::Matrix<double, 7, 1> q_upper_;
            Eigen::Matrix<double, 7, 1> q_lower_;
            Eigen::Matrix<double, 7, 1> qdot_upper_;
            Eigen::Matrix<double, 7, 1> qdot_lower_;
            Eigen::Matrix<double, 7, 1> qddot_upper_;
            Eigen::Matrix<double, 7, 1> qddot_lower_;
            Eigen::Matrix<double, 7, 7> error_weight_;
            Eigen::Matrix<double, 7, 7> damping_weight_;
            
            void setJointLimit(const std::string &file_path);
            void setWeightMatrix(const std::string &file_path);
            void setCost() override;
            void setConstraint() override;
    };
} // namespace QP
#endif