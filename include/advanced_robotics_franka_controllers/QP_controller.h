#ifndef QP_CONTROLLER_H
#define QP_CONTROLLER_H

#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "OsqpEigen/OsqpEigen.h"

#include "advanced_robotics_franka_controllers/robot_model.h"
#include "suhan_benchmark.h"
#include "math_type_define.h"

using json = nlohmann::json;
static const std::string pkg_path =  std::string(BUILD_DIRECTORY) + "/";

namespace QP_CONTROLLER
{
    enum Status
    {
        SOLVED,
        QP_DualInfeasibleInaccurate,
        QP_PrimalInfeasibleInaccurate,
        QP_SolvedInaccurate,
        QP_MaxIterReached,
        QP_PrimalInfeasible,
        QP_DualInfeasible,
        Sigint, 
        INVALID_SETTINGS,
        NAN_HESSIAN,
        NON_PD_HESSIAN
    };

    struct TimeDuration
    {
        double set_qp;
        double set_solver;
        double solve_qp;
        void setZero(){set_qp=0; set_solver=0; solve_qp=0;}
    };
    
    struct QPIndex
    {
        int s1 = 0;
        int s2 = 1;
        int s3 = 2;
        int s4 = 3;
        int s5 = 4;
        int s6 = 5;

        int q1 = 6;
        int q2 = 7;
        int q3 = 8;
        int q4 = 9;
        int q5 = 10;
        int q6 = 11;
        int q7 = 12;

        int con_slack = 0; // 0-5
        int con_q = 6; // 6-12
        int con_qdot = 13; // 13-19
        int con_qddot = 20; // 20-26
    };

    static const int nq = 7; // 7 for panda dof
    static const int ns = 6; // 6 for end-effector dof
    static const int nx = ns + nq;
    static const int nc = 6 + 7*3; // number of constraints
    static const QPIndex si_index;

    class QP
    {
        public:
            QP();
            ~QP();
            void setCurrentState(const Eigen::Matrix<double, 7, 1> &q_current, const Eigen::Matrix<double, 7, 1> &qdot_current, const Eigen::Matrix<double, 6, 7> &j_current)
            {
                q_current_ = q_current;
                qdot_current_ = qdot_current;
                j_current_ = j_current;
                // j_current_ = robot_model_.getJacobian(q_current_);
            }
            void setDesiredEEVel(const Eigen::Matrix<double, 6, 1> &xdot_desired)
            {
                xdot_desired_ = xdot_desired;
            }
            bool solveQP(Eigen::Matrix<double, 7, 1> &opt_qdot, TimeDuration &time_status);

        private:
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

            RobotModel robot_model_;
            // OsqpEigen::Solver solver_;

            bool is_first_;

            OsqpEigen::Status qp_status_;


            void setJointLimit(const std::string &file_path);
            void setWeightMatrix(const std::string &file_path);
    };
}

#endif