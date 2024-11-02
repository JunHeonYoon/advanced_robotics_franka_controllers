#include "advanced_robotics_franka_controllers/QP_cartesian_velocity.h"

namespace QP
{
    CartesianVelocity::CartesianVelocity()
    :Base()
    {
        Base::setQPsize(ns_ + nq_, ns_ + nq_ * 3);

        q_upper_.setZero();
        q_lower_.setZero();
        qdot_upper_.setZero();
        qdot_lower_.setZero();
        qddot_upper_.setZero();
        qddot_lower_.setZero();
        slack_weight_.setZero();
        damping_weight_.setZero();

        setJointLimit(pkg_path + "Params/QP_cartesian_velocity/bounds.json");
        setWeightMatrix(pkg_path + "Params/QP_cartesian_velocity/weights.json");

        std::cout << "======================================================================" << std::endl;
        std::cout << "=================== QP Cartesian Velocity loaded!! ===================" << std::endl;
        std::cout << "======================================================================" << std::endl;
    }

    void CartesianVelocity::setCurrentState(const Eigen::Matrix<double, 7, 1> &q_current, const Eigen::Matrix<double, 7, 1> &qdot_current, const Eigen::Matrix<double, 6, 7> &j_current)
    {
        q_current_ = q_current;
        qdot_current_ = qdot_current;
        j_current_ = j_current;
        // j_current_ = robot_model_.getJacobian(q_current_);
    }
    
    void CartesianVelocity::setDesiredEEVel(const Eigen::Matrix<double, 6, 1> &xdot_desired)
    {
        xdot_desired_ = xdot_desired;
    }

    void CartesianVelocity::setJointLimit(const std::string &file_path)
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

    void CartesianVelocity::setWeightMatrix(const std::string &file_path)
    {
        std::ifstream iWeights(file_path);
        json jsonWeight;
        iWeights >> jsonWeight;

        slack_weight_(0,0) = jsonWeight["slack1"];
        slack_weight_(1,1) = jsonWeight["slack2"];
        slack_weight_(2,2) = jsonWeight["slack3"];
        slack_weight_(3,3) = jsonWeight["slack4"];
        slack_weight_(4,4) = jsonWeight["slack5"];
        slack_weight_(5,5) = jsonWeight["slack6"];

        damping_weight_(0,0) = jsonWeight["damping1"];
        damping_weight_(1,1) = jsonWeight["damping2"];
        damping_weight_(2,2) = jsonWeight["damping3"];
        damping_weight_(3,3) = jsonWeight["damping4"];
        damping_weight_(4,4) = jsonWeight["damping5"];
        damping_weight_(5,5) = jsonWeight["damping6"];
        damping_weight_(6,6) = jsonWeight["damping7"];

        mani_weight_ = jsonWeight["mani"];
    }

    void CartesianVelocity::setCost()
    {
        P_ds_.block(si_index_.s1,  si_index_.s1,  ns_, ns_) = 2.0 * slack_weight_;
        P_ds_.block(si_index_.dq1, si_index_.dq1, nq_, nq_) = 2.0 * damping_weight_;

        double mani = robot_model_.getManipulability(q_current_);
        double mani_cubic_weight;
        if(mani > 0.05) 
        {
            mani_cubic_weight = 0.0;
        }
        else if(mani < 0.01)
        {
            mani_cubic_weight = mani_weight_;
        }
        else
        {
            mani_cubic_weight = DyrosMath::cubic(mani, 0.01, 0.05, mani_weight_, 0., 0., 0.);
        }
        q_ds_.block(si_index_.dq1, 0, nq_, 1) = -mani_cubic_weight * robot_model_.getDManipulability(q_current_);
    }

    void CartesianVelocity::setConstraint()
    {
        A_ds_.block(si_index_.con_slack, si_index_.s1,  ns_, ns_).setIdentity(ns_, ns_);
        A_ds_.block(si_index_.con_slack, si_index_.dq1, ns_, nq_) = j_current_;
        A_ds_.block(si_index_.con_q,     si_index_.dq1, nq_, nq_).setIdentity(nq_, nq_);
        A_ds_.block(si_index_.con_qdot,  si_index_.dq1, nq_, nq_).setIdentity(nq_, nq_);
        A_ds_.block(si_index_.con_qddot, si_index_.dq1, nq_, nq_).setIdentity(nq_, nq_);

        l_ds_.block(si_index_.con_slack, 0, ns_, 1) = xdot_desired_;
        l_ds_.block(si_index_.con_q,     0, nq_, 1) = hz_ * (q_lower_ - q_current_);
        l_ds_.block(si_index_.con_qdot,  0, nq_, 1) = qdot_lower_;
        l_ds_.block(si_index_.con_qddot, 0, nq_, 1) = qddot_lower_ / hz_ + qdot_current_;
        
        u_ds_.block(si_index_.con_slack, 0, ns_, 1) = xdot_desired_;
        u_ds_.block(si_index_.con_q,     0, nq_, 1) = hz_ * (q_upper_ - q_current_);
        u_ds_.block(si_index_.con_qdot,  0, nq_, 1) = qdot_upper_;
        u_ds_.block(si_index_.con_qddot, 0, nq_, 1) = qddot_upper_ / hz_ + qdot_current_;
    }

    bool CartesianVelocity::getOptJointVel(Eigen::Matrix<double, 7, 1> &opt_qdot, TimeDuration &time_status)
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
