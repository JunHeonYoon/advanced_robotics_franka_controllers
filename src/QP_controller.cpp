#include "advanced_robotics_franka_controllers/QP_controller.h"

namespace QP_CONTROLLER
{
    QP::QP()
    {
        q_upper_.setZero();
        q_lower_.setZero();
        qdot_upper_.setZero();
        qdot_lower_.setZero();
        qddot_upper_.setZero();
        qddot_lower_.setZero();
        slack_weight_.setZero();
        damping_weight_.setZero();

        setJointLimit(pkg_path + "Params/bounds.json");
        setWeightMatrix(pkg_path + "Params/weight.json");

        is_first_ = true;
    }

    QP::~QP()
    {

    }

     void QP::setJointLimit(const std::string &file_path)
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

        q_upper_(0) *= 0.9;
        q_upper_(1) *= 0.9;
        q_upper_(2) *= 0.9;
        q_upper_(3) *= 0.9;
        q_upper_(4) *= 0.9;
        q_upper_(5) *= 0.9;
        q_upper_(6) *= 0.9;

        q_lower_(0) *= 0.9;
        q_lower_(1) *= 0.9;
        q_lower_(2) *= 0.9;
        q_lower_(3) *= 0.9;
        q_lower_(4) *= 0.9;
        q_lower_(5) *= 0.9;
        q_lower_(6) *= 0.9;

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

        qddot_upper_(0) *= 0.3;
        qddot_upper_(1) *= 0.3;
        qddot_upper_(2) *= 0.3;
        qddot_upper_(3) *= 0.3;
        qddot_upper_(4) *= 0.3;
        qddot_upper_(5) *= 0.3;
        qddot_upper_(6) *= 0.3;

        qddot_lower_(0) *= 0.3;
        qddot_lower_(1) *= 0.3;
        qddot_lower_(2) *= 0.3;
        qddot_lower_(3) *= 0.3;
        qddot_lower_(4) *= 0.3;
        qddot_lower_(5) *= 0.3;
        qddot_lower_(6) *= 0.3;
    }

    void QP::setWeightMatrix(const std::string &file_path)
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

    bool QP::solveQP(Eigen::Matrix<double, 7, 1> &opt_qdot)
    {
        Eigen::Matrix<double, nx, nx> P_ds;
        P_ds.setZero();
        P_ds.block(si_index.s1, si_index.s1, ns, ns) = 2.0 * slack_weight_;
        P_ds.block(si_index.q1, si_index.q1, nq, nq) = 2.0 * damping_weight_;

        Eigen::Matrix<double, nx, 1> q_ds;
        q_ds.setZero();
        q_ds.block(si_index.q1, 0, nq, 1) = -mani_weight_ * robot_model_.getDManipulability(q_current_);

        Eigen::Matrix<double, nc, nx> A_ds;
        A_ds.setZero();
        A_ds.block(si_index.con_slack, si_index.s1, ns, ns).setIdentity(ns, ns);
        A_ds.block(si_index.con_slack, si_index.q1, ns, nq) = j_current_;
        A_ds.block(si_index.con_q, si_index.q1, nq, nq).setIdentity(nq, nq);
        A_ds.block(si_index.con_qdot, si_index.q1, nq, nq).setIdentity(nq, nq);
        A_ds.block(si_index.con_qddot, si_index.q1, nq, nq).setIdentity(nq, nq);

        Eigen::Matrix<double, nc, 1> l_ds;
        l_ds.setZero();
        l_ds.block(si_index.con_slack, 0, ns, 1) = xdot_desired_;
        l_ds.block(si_index.con_q, 0, nq, 1) = hz_ * (q_lower_ - q_current_);
        l_ds.block(si_index.con_qdot, 0, nq, 1) = qdot_lower_;
        l_ds.block(si_index.con_qddot, 0, nq, 1) = qddot_lower_ / hz_ + qdot_current_;
        
        Eigen::Matrix<double, nc, 1> u_ds;
        u_ds.setZero();
        u_ds.block(si_index.con_slack, 0, ns, 1) = xdot_desired_;
        u_ds.block(si_index.con_q, 0, nq, 1) = hz_ * (q_upper_ - q_current_);
        u_ds.block(si_index.con_qdot, 0, nq, 1) = qdot_upper_;
        u_ds.block(si_index.con_qddot, 0, nq, 1) = qddot_upper_ / hz_ + qdot_current_;
        
        /* 
        min   1/2 x' P x + q' x
         x

        subject to
        l <= A x <= u

        with :
        P sparse (n x n) positive definite
        q dense  (n x 1)
        A sparse (nc x n)
        l dense (nc x 1)
        u dense (nc x 1)
        (n = dof + ee_dof, nc = 2*ee_dof + dof + num_links)
        */
        Eigen::SparseMatrix<double> P(nx, nx);
        Eigen::Matrix<double, nx, 1> q;
        Eigen::SparseMatrix<double> A(nc, nx);
        Eigen::Matrix<double, nc, 1> l, u;
        P = P_ds.sparseView();
        A = A_ds.sparseView();
        q = q_ds;
        l = l_ds;
        u = u_ds;

        OsqpEigen::Solver solver_;

        // settings
        solver_.settings()->setWarmStart(false);
        solver_.settings()->getSettings()->eps_abs = 1e-4;
        solver_.settings()->getSettings()->eps_rel = 1e-5;
        solver_.settings()->getSettings()->verbose = false;

        // set the initial data of the QP solver
        solver_.data()->setNumberOfVariables(nx);
        solver_.data()->setNumberOfConstraints(nc);
        if (!solver_.data()->setHessianMatrix(P))           return false;
        if (!solver_.data()->setGradient(q))                return false;
        if (!solver_.data()->setLinearConstraintsMatrix(A)) return false;
        if (!solver_.data()->setLowerBound(l))              return false;
        if (!solver_.data()->setUpperBound(u))              return false;

        // instantiate the solver
        if (!solver_.initSolver()) return false;

        // solve the QP problem
        if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
        qp_status_ = solver_.getStatus();
        if (solver_.getStatus() != OsqpEigen::Status::Solved) return false;
        // if (solver_.getStatus() != OsqpEigen::Status::Solved && solver_.getStatus() != OsqpEigen::Status::SolvedInaccurate) return false;

        auto sol = solver_.getSolution();
        opt_qdot = sol.segment(si_index.q1, nq);

        solver_.clearSolverVariables();
        solver_.clearSolver();

        return true;
    }
}