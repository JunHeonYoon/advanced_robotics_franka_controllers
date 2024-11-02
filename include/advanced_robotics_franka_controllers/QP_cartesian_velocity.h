#ifndef QP_CARTESIAN_VELOCITY_H
#define QP_CARTESIAN_VELOCITY_H

#include "advanced_robotics_franka_controllers/QP_base.h"
#include <nlohmann/json.hpp>
#include "math_type_define.h"

using json = nlohmann::json;

namespace QP
{
    class CartesianVelocity : public Base
    {
        public:
            CartesianVelocity();
            void setCurrentState(const Eigen::Matrix<double, 7, 1> &q_current, const Eigen::Matrix<double, 7, 1> &qdot_current, const Eigen::Matrix<double, 6, 7> &j_current);
            void setDesiredEEVel(const Eigen::Matrix<double, 6, 1> &xdot_desired);
            bool getOptJointVel(Eigen::Matrix<double, 7, 1> &opt_qdot, TimeDuration &time_status);

        private:
            int nq_ = 7; // 7 for panda dof
            int ns_ = 6; // 6 for end-effector dofS
            struct QPIndex
            {
                int s1 = 0;
                int s2 = 1;
                int s3 = 2;
                int s4 = 3;
                int s5 = 4;
                int s6 = 5;

                int dq1 = 6;
                int dq2 = 7;
                int dq3 = 8;
                int dq4 = 9;
                int dq5 = 10;
                int dq6 = 11;
                int dq7 = 12;

                int con_slack = 0;  // 0-5
                int con_q = 6;      // 6-12
                int con_qdot = 13;  // 13-19
                int con_qddot = 20; // 20-26
            }si_index_;

            Eigen::Matrix<double, 7, 1> q_current_;
            Eigen::Matrix<double, 7, 1> qdot_current_;
            Eigen::Matrix<double, 6, 7> j_current_;
            Eigen::Matrix<double, 6, 1> xdot_desired_;

            const double hz_ = 200.;

            Eigen::Matrix<double, 7, 1> q_upper_;
            Eigen::Matrix<double, 7, 1> q_lower_;
            Eigen::Matrix<double, 7, 1> qdot_upper_;
            Eigen::Matrix<double, 7, 1> qdot_lower_;
            Eigen::Matrix<double, 7, 1> qddot_upper_;
            Eigen::Matrix<double, 7, 1> qddot_lower_;
            Eigen::Matrix<double, 6, 6> slack_weight_;
            Eigen::Matrix<double, 7, 7> damping_weight_;
            double mani_weight_;
            
            void setJointLimit(const std::string &file_path);
            void setWeightMatrix(const std::string &file_path);
            void setCost() override;
            void setConstraint() override;
    };
} // namespace QP
#endif