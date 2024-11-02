#include "advanced_robotics_franka_controllers/QP_base.h"

namespace QP
{
    Base::Base()
    {

    }

    void Base::setQPsize(const int &nx, const int &nc)
    {
        nx_ = nx;
        nc_ = nc;
        time_status_.setZero();

        P_ds_.setZero(nx_, nx_);
        q_ds_.setZero(nx_);
        A_ds_.setZero(nc_, nx_);
        l_ds_.setZero(nc_);
        u_ds_.setZero(nc_);
    }


    bool Base::solveQP(Eigen::MatrixXd &sol, TimeDuration &time_status)
    {
        timer_.reset();
        setCost();
        setConstraint();
        time_status.set_qp = timer_.elapsedAndReset();

        /* 
        min   1/2 x' P x + q' x
         x

        subject to
        l <= A x <= u

        with :
        P sparse (nx x nx) positive definite
        q dense  (nx x 1)
        A sparse (nc x n)
        l dense (nc x 1)
        u dense (nc x 1)
        */
        Eigen::SparseMatrix<double> P(nx_, nx_);
        Eigen::VectorXd q;
        Eigen::SparseMatrix<double> A(nc_, nx_);
        Eigen::VectorXd l, u;
        P = P_ds_.sparseView();
        A = A_ds_.sparseView();
        q = q_ds_;
        l = l_ds_;
        u = u_ds_;

        OsqpEigen::Solver solver;

        // settings
        solver.settings()->setWarmStart(false);
        solver.settings()->getSettings()->eps_abs = 1e-4;
        solver.settings()->getSettings()->eps_rel = 1e-5;
        solver.settings()->getSettings()->verbose = false;

        // set the initial data of the QP solver
        solver.data()->setNumberOfVariables(nx_);
        solver.data()->setNumberOfConstraints(nc_);
        if (!solver.data()->setHessianMatrix(P))           return false;
        if (!solver.data()->setGradient(q))                return false;
        if (!solver.data()->setLinearConstraintsMatrix(A)) return false;
        if (!solver.data()->setLowerBound(l))              return false;
        if (!solver.data()->setUpperBound(u))              return false;


        // instantiate the solver
        if (!solver.initSolver()) return false;

        // solve the QP problem
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
        qp_status_ = solver.getStatus();
        if (solver.getStatus() != OsqpEigen::Status::Solved) return false;
        // if (solver.getStatus() != OsqpEigen::Status::Solved && solver.getStatus() != OsqpEigen::Status::SolvedInaccurate) return false;

        time_status.set_solver = timer_.elapsedAndReset();

        sol = solver.getSolution();

        time_status.solve_qp = timer_.elapsedAndReset();

        solver.clearSolverVariables();
        solver.clearSolver();

        return true;
    }
} // namespace QP