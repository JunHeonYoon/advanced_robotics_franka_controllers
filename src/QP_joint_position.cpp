#include "advanced_robotics_franka_controllers/QP_joint_position.h"

namespace QP
{
    JointPosition::JointPosition()
    :Base()
    {
        Base::setQPsize(2 *nq_, 4 * nq_);
        
        q_upper_.setZero();
        q_lower_.setZero();
        qdot_upper_.setZero();
        qdot_lower_.setZero();
        qddot_upper_.setZero();
        qddot_lower_.setZero();
        error_weight_.setZero();
        damping_weight_.setZero();

        setJointLimit(pkg_path + "Params/QP_joint_position/bounds.json");
        setWeightMatrix(pkg_path + "Params/QP_joint_position/weights.json");

        std::cout << "======================================================================" << std::endl;
        std::cout << "===================== QP Joint Position loaded!! =====================" << std::endl;
        std::cout << "======================================================================" << std::endl;
    }

    void JointPosition::setCurrentState(const Eigen::Matrix<double, 7, 1> &q_current, const Eigen::Matrix<double, 7, 1> &qdot_current)
    {
        q_current_ = q_current;
        qdot_current_ = qdot_current;
    }
    
    void JointPosition::setDesiredJointPosition(const Eigen::Matrix<double, 7, 1> &q_desired)
    {
        q_desired_ = q_desired;
    }

    void JointPosition::setJointLimit(const std::string &file_path)
    {
        std::ifstream iBounds(file_path);
        json jsonBounds;
        iBounds >> jsonBounds;

        q_upper_(0) = jsonBounds["q1u"];
        q_upper_(1) = jsonBounds["q2u"];
        q_upper_(2) = jsonBounds["q3u"];
        q_upper_(3) = jsonBounds["q4u"];
        q_upper_(4) = jsonBounds["q5u"];
        q_upper_(5) = jsonBounds["q6u"];
        q_upper_(6) = jsonBounds["q7u"];

        q_lower_(0) = jsonBounds["q1l"];
        q_lower_(1) = jsonBounds["q2l"];
        q_lower_(2) = jsonBounds["q3l"];
        q_lower_(3) = jsonBounds["q4l"];
        q_lower_(4) = jsonBounds["q5l"];
        q_lower_(5) = jsonBounds["q6l"];
        q_lower_(6) = jsonBounds["q7l"];

        qdot_upper_(0) = jsonBounds["dq1u"];
        qdot_upper_(1) = jsonBounds["dq2u"];
        qdot_upper_(2) = jsonBounds["dq3u"];
        qdot_upper_(3) = jsonBounds["dq4u"];
        qdot_upper_(4) = jsonBounds["dq5u"];
        qdot_upper_(5) = jsonBounds["dq6u"];
        qdot_upper_(6) = jsonBounds["dq7u"];

        qdot_lower_(0) = jsonBounds["dq1l"];
        qdot_lower_(1) = jsonBounds["dq2l"];
        qdot_lower_(2) = jsonBounds["dq3l"];
        qdot_lower_(3) = jsonBounds["dq4l"];
        qdot_lower_(4) = jsonBounds["dq5l"];
        qdot_lower_(5) = jsonBounds["dq6l"];
        qdot_lower_(6) = jsonBounds["dq7l"];

        qddot_upper_(0) = jsonBounds["ddq1u"];
        qddot_upper_(1) = jsonBounds["ddq2u"];
        qddot_upper_(2) = jsonBounds["ddq3u"];
        qddot_upper_(3) = jsonBounds["ddq4u"];
        qddot_upper_(4) = jsonBounds["ddq5u"];
        qddot_upper_(5) = jsonBounds["ddq6u"];
        qddot_upper_(6) = jsonBounds["ddq7u"];

        qddot_lower_(0) = jsonBounds["ddq1l"];
        qddot_lower_(1) = jsonBounds["ddq2l"];
        qddot_lower_(2) = jsonBounds["ddq3l"];
        qddot_lower_(3) = jsonBounds["ddq4l"];
        qddot_lower_(4) = jsonBounds["ddq5l"];
        qddot_lower_(5) = jsonBounds["ddq6l"];
        qddot_lower_(6) = jsonBounds["ddq7l"];

        q_upper_(0) *= 1.0;
        q_upper_(1) *= 1.0;
        q_upper_(2) *= 1.0;
        q_upper_(3) *= 1.0;
        q_upper_(4) *= 1.0;
        q_upper_(5) *= 1.0;
        q_upper_(6) *= 1.0;

        q_lower_(0) *= 1.0;
        q_lower_(1) *= 1.0;
        q_lower_(2) *= 1.0;
        q_lower_(3) *= 1.0;
        q_lower_(4) *= 1.0;
        q_lower_(5) *= 1.0;
        q_lower_(6) *= 1.0;

        qdot_upper_(0) *= 0.5;
        qdot_upper_(1) *= 0.5;
        qdot_upper_(2) *= 0.5;
        qdot_upper_(3) *= 0.5;
        qdot_upper_(4) *= 0.5;
        qdot_upper_(5) *= 0.5;
        qdot_upper_(6) *= 0.5;

        qdot_lower_(0) *= 0.5;
        qdot_lower_(1) *= 0.5;
        qdot_lower_(2) *= 0.5;
        qdot_lower_(3) *= 0.5;
        qdot_lower_(4) *= 0.5;
        qdot_lower_(5) *= 0.5;
        qdot_lower_(6) *= 0.5;

        qddot_upper_(0) *= 0.1;
        qddot_upper_(1) *= 0.1;
        qddot_upper_(2) *= 0.1;
        qddot_upper_(3) *= 0.1;
        qddot_upper_(4) *= 0.1;
        qddot_upper_(5) *= 0.1;
        qddot_upper_(6) *= 0.1;

        qddot_lower_(0) *= 0.1;
        qddot_lower_(1) *= 0.1;
        qddot_lower_(2) *= 0.1;
        qddot_lower_(3) *= 0.1;
        qddot_lower_(4) *= 0.1;
        qddot_lower_(5) *= 0.1;
        qddot_lower_(6) *= 0.1;
    }

    void JointPosition::setWeightMatrix(const std::string &file_path)
    {
        std::ifstream iWeights(file_path);
        json jsonWeight;
        iWeights >> jsonWeight;

        error_weight_(0,0) = jsonWeight["q_error1"];
        error_weight_(1,1) = jsonWeight["q_error2"];
        error_weight_(2,2) = jsonWeight["q_error3"];
        error_weight_(3,3) = jsonWeight["q_error4"];
        error_weight_(4,4) = jsonWeight["q_error5"];
        error_weight_(5,5) = jsonWeight["q_error6"];
        error_weight_(6,6) = jsonWeight["q_error7"];

        damping_weight_(0,0) = jsonWeight["damping1"];
        damping_weight_(1,1) = jsonWeight["damping2"];
        damping_weight_(2,2) = jsonWeight["damping3"];
        damping_weight_(3,3) = jsonWeight["damping4"];
        damping_weight_(4,4) = jsonWeight["damping5"];
        damping_weight_(5,5) = jsonWeight["damping6"];
        damping_weight_(6,6) = jsonWeight["damping7"];
    }

    void JointPosition::setCost()
    {
        P_ds_.block(si_index_.dq1, si_index_.dq1, nq_, nq_) = 2.0 * damping_weight_;
        P_ds_.block(si_index_.qe1, si_index_.qe1, nq_, nq_) = 2.0 * error_weight_;
    }

    void JointPosition::setConstraint()
    {
        const double dt = 1. / hz_;

        A_ds_.block(si_index_.con_error, si_index_.dq1, nq_, nq_) = Eigen::MatrixXd::Identity(nq_, nq_) * dt;
        A_ds_.block(si_index_.con_error, si_index_.qe1, nq_, nq_) = Eigen::MatrixXd::Identity(nq_, nq_);
        A_ds_.block(si_index_.con_q,     si_index_.dq1, nq_, nq_) = Eigen::MatrixXd::Identity(nq_, nq_) * dt;
        A_ds_.block(si_index_.con_qdot,  si_index_.dq1, nq_, nq_) = Eigen::MatrixXd::Identity(nq_, nq_);
        A_ds_.block(si_index_.con_qddot, si_index_.dq1, nq_, nq_) = Eigen::MatrixXd::Identity(nq_, nq_) / dt;

        l_ds_.block(si_index_.con_error, 0, nq_, 1) = q_desired_ - q_current_;
        l_ds_.block(si_index_.con_q,     0, nq_, 1) = q_lower_ - q_current_;
        l_ds_.block(si_index_.con_qdot,  0, nq_, 1) = qdot_lower_;
        l_ds_.block(si_index_.con_qddot, 0, nq_, 1) = qddot_lower_ + qdot_current_ / dt;
        
        u_ds_.block(si_index_.con_error, 0, nq_, 1) = q_desired_ - q_current_;
        u_ds_.block(si_index_.con_q,     0, nq_, 1) = q_upper_ - q_current_;
        u_ds_.block(si_index_.con_qdot,  0, nq_, 1) = qdot_upper_;
        u_ds_.block(si_index_.con_qddot, 0, nq_, 1) = qddot_upper_ + qdot_current_ / dt;
    }

    bool JointPosition::getOptJointVel(Eigen::Matrix<double, 7, 1> &opt_qdot, TimeDuration &time_status)
    {
        Eigen::MatrixXd sol;
        bool status = solveQP(sol, time_status);
        if(status == true)
        {
            opt_qdot = sol.block(si_index_.dq1, 0, nq_, 1);
        }
        else
        {
            opt_qdot.setZero(nq_, 1);
        }
        return status;
    }
} // namespace QP
