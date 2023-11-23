#include <advanced_robotics_franka_controllers/NJSDF.h>

namespace NJSDF
{
    // ------------------------------------- QP ---------------------------------------------
    QP::QP(double obs_radius, int hz, barrier_func_type barrier_func)
    {
        obs_radius_ = obs_radius;
        hz_ = hz;
        barrier_func_ = barrier_func;

        njsdf_model_ = std::make_shared<NNmodel>();
        Eigen::Matrix<double, 4, 1> n_hidden;
        n_hidden << 256, 256, 256, 256;
        njsdf_model_->setNeuralNetwork(dof+3, num_links, n_hidden, true);

        Eigen::Matrix<double, dof, 1> joint_upper_lim, joint_lower_lim;
        joint_lower_lim << -2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973;
        joint_upper_lim << 2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973;
        setJointLimit(joint_lower_lim, joint_upper_lim);

        Eigen::Matrix<double, ee_dof, 1> vel_upper_lim, vel_lower_lim;
        vel_upper_lim << 0.5, 0.5, 0.5, M_PI/2, M_PI/2, M_PI/2;
        vel_lower_lim = -vel_upper_lim;
        setEEVelocityLimit(vel_lower_lim, vel_upper_lim);

        Eigen::Matrix<double, ee_dof,1> slack_weight_diag;
        Eigen::Matrix<double, dof, 1> damping_weight_diag;
        slack_weight_diag << 1, 1, 1, 1, 1, 1;
        damping_weight_diag << 10, 10, 10, 10, 10, 10, 10;
        setWeightMatrix(slack_weight_diag.asDiagonal(), damping_weight_diag.asDiagonal());

        is_first_ = true;
        
    }

    QP::~QP()
    {
        std::cout << "QP terminate" <<std::endl;
    }
    
    void QP::setJointLimit(Eigen::Matrix<double, dof, 1> lower_lim, Eigen::Matrix<double, dof, 1> upper_lim)
    {
        joint_lower_lim_ = lower_lim;
        joint_upper_lim_ = upper_lim;
    }
    
    void QP::setEEVelocityLimit(Eigen::Matrix<double, ee_dof, 1> lower_lim, Eigen::Matrix<double, ee_dof, 1> upper_lim)
    {
        ee_vel_lower_lim_ = lower_lim;
        ee_vel_upper_lim_ = upper_lim;
    }

    void QP::setWeightMatrix(Eigen::Matrix<double, ee_dof, ee_dof> slack_weight, Eigen::Matrix<double, dof, dof> damping_weight)
    {
        slack_weight_ = slack_weight;
        damping_weight_ = damping_weight;
    }
    
    void QP::setCurrentState(Eigen::Affine3d ee_pose, Eigen::Matrix<double, dof, 1> joint, Eigen::Matrix<double, ee_dof, dof> jacobian)
    {
        x_ = ee_pose;
        q_ =joint;
        j_ = jacobian;
    }

    void QP::setDesiredState(Eigen::Affine3d desired_ee_pose)
    {
        desired_x_ = desired_ee_pose;
    }

    void QP::setObsPosition(Eigen::Matrix<double,3,1> obs_position)
    {
        obs_position_ = obs_position;
    }

    std::pair<Eigen::Matrix<double, num_links, 1>, Eigen::Matrix<double, num_links, dof>> QP::getSD()
    {
        Eigen::Matrix<double, dof+3, 1> input;
        input << q_, obs_position_;
        auto y_pred = njsdf_model_->calculateMlpOutput(input, false);
        // std::cout<<y_pred.first.transpose()<<std::endl;
        return std::make_pair(y_pred.first, y_pred.second.block<num_links, dof>(0, 0));
    }

    bool QP::solveQP(bool verbose)
    {
         // settings
        if (!verbose)
            solver_.settings()->setVerbosity(false);
        // solver_.settings()->setPrimalInfeasibilityTolerance(1e-4);
        solver_.settings()->setWarmStart(true);

        // get predicted minimum distance vector and its jacobian by NN model 
        auto y_pred = QP::getSD();
        pred_min_dist_ = y_pred.first;
        pred_min_dist_grad_ = y_pred.second;
        // std::cout<<pred_min_dist_.size()<<std::endl;
        // std::cout<<pred_min_dist_grad_.rows() << pred_min_dist_grad_.cols() <<std::endl;

        // get ee pose error (with error gain)
        Eigen::Matrix<double, ee_dof, 1> x_error;
        double posi_gain = 5, rot_gain = 1;
        x_error.head(3) = (desired_x_.translation() - x_.translation()) * posi_gain;
        x_error.tail(3) = DyrosMath::getPhi(desired_x_.rotation(), x_.rotation()) * rot_gain;


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
       
        const int n  = ee_dof + dof;
        const int nc = 2*ee_dof + dof + num_links;

        Eigen::SparseMatrix<double> P(n, n);
        Eigen::Matrix<double, n, 1> q;
        Eigen::SparseMatrix<double> A(nc, n);
        Eigen::Matrix<double, nc, 1> l, u;

        if (is_first_)
        {
            is_first_ = false;
            
            // for hessian (P)
            for (size_t i=0; i<n; i++)
            {
                double value;
                if (i < ee_dof) value = slack_weight_.diagonal()[i];
                else            value = damping_weight_.diagonal()[i - ee_dof];

                if (value != 0) P.insert(i, i) = value;
            }

            // for gradient (q)
            q.setZero();

            // for constraint matrix (A)
            for (size_t i=0; i<ee_dof; i++) A.insert(i, i) = -1.0;
            for (size_t i=ee_dof; i<(ee_dof+dof); i++) A.insert(i, i) = 1.0;
            for (size_t i=0; i<ee_dof; i++)
            {
                for (size_t j=0; j<dof; j++)
                {
                    double value = j_(i, j);
                    if (std::fabs(value) > 1e-8)
                    {
                        A.insert(i, j+ee_dof) = value;
                        A.insert(ee_dof+dof+num_links+i, j+ee_dof) = value*hz_;
                    }
                }
            }
            for (size_t i=0; i<num_links; i++)
            {
                for (size_t j=0; j<dof; j++)
                {
                    double value = -pred_min_dist_grad_(i, j);
                    if (std::fabs(value) > 1e-8)
                    {
                        A.insert(ee_dof+dof+i, ee_dof+j) = value;
                    }
                }
            }

            // for constraint vectors (l, u)
            l.setZero();
            u.setZero();
            l << x_error, joint_lower_lim_ - q_, -OsqpEigen::INFTY * Eigen::Matrix<double, num_links, 1>::Ones(), ee_vel_lower_lim_;
            switch (barrier_func_)
            {
            case LOG:
                u << x_error, joint_upper_lim_ - q_, (pred_min_dist_.array() - obs_radius_*100.0*1.3).log().matrix(), ee_vel_upper_lim_;
                break;
            case LINEAR:
                u << x_error, joint_upper_lim_ - q_, (pred_min_dist_.array() - obs_radius_*100.0*1.3).matrix(), ee_vel_upper_lim_;
                break;
            case RBF:
                u << x_error, joint_upper_lim_ - q_, -RBFunc(0.5, 1.0, pred_min_dist_ - obs_radius_*100.0*1.3*Eigen::Matrix<double, num_links, 1>::Ones()), ee_vel_upper_lim_;
                break;
            default:
                std::cout << "NOT proper barrier function" <<std::endl;
                return false;
                break;
            }


            // set the initial data of the QP solver
            solver_.data()->setNumberOfVariables(n);
            solver_.data()->setNumberOfConstraints(nc);
            if (!solver_.data()->setHessianMatrix(P)) 
            {
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "Did not set proper Hessian Matrix"<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                return false;
            }
            if (!solver_.data()->setGradient(q))
            {
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "Did not set proper Gradient Matrix"<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                return false;
            }
            if (!solver_.data()->setLinearConstraintsMatrix(A))
            {
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "Did not set proper Linear constraint Matrix"<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                return false;
            }
            if (!solver_.data()->setLowerBound(l))
            {
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "Did not set proper Lower Bound"<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                return false;
            }
            if (!solver_.data()->setUpperBound(u))
            {
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "Did not set proper Upper Bound"<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                return false;
            }

            // instantiate the solver
            if (!solver_.initSolver())
            {
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "Did not Init solver"<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                return false;
            }
        }
        
        else
        {
            // for constraint matrix (A)
            for (size_t i=0; i<ee_dof; i++) A.insert(i, i) = -1.0;
            for (size_t i=ee_dof; i<(ee_dof+dof); i++) A.insert(i, i) = 1.0;
            for (size_t i=0; i<ee_dof; i++)
            {
                for (size_t j=0; j<dof; j++)
                {
                    double value = j_(i, j);
                    if (std::fabs(value) > 1e-8)
                    {
                        A.insert(i, j+ee_dof) = value;
                        A.insert(ee_dof+dof+num_links+i, j+ee_dof) = value*hz_;
                    }
                }
            }
            for (size_t i=0; i<num_links; i++)
            {
                for (size_t j=0; j<dof; j++)
                {
                    double value = -pred_min_dist_grad_(i, j);
                    if (std::fabs(value) > 1e-8)
                    {
                        A.insert(ee_dof+dof+i, ee_dof+j) = value;
                    }
                }
            }

            // for constraint vectors (l, u)
            l.setZero();
            u.setZero();
            l << x_error, joint_lower_lim_ - q_, -OsqpEigen::INFTY * Eigen::Matrix<double, num_links, 1>::Ones(), ee_vel_lower_lim_;
            switch (barrier_func_)
            {
            case LOG:
                u << x_error, joint_upper_lim_ - q_, (pred_min_dist_.array() - obs_radius_*100.0*1.3).log().matrix(), ee_vel_upper_lim_;
                break;
            case LINEAR:
                u << x_error, joint_upper_lim_ - q_, (pred_min_dist_.array() - obs_radius_*100.0*1.3).matrix(), ee_vel_upper_lim_;
                break;
            case RBF:
                u << x_error, joint_upper_lim_ - q_, -RBFunc(0.5, 1.0, pred_min_dist_ - obs_radius_*100.0*1.3*Eigen::Matrix<double, num_links, 1>::Ones()), ee_vel_upper_lim_;
                break;
            default:
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "NOT proper barrier function" <<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                return false;
                break;
            }

            if (!solver_.updateBounds(l, u))
            {
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "Did not update proper Bound"<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                return false;
            }
            if (!solver_.updateLinearConstraintsMatrix(A))
            {
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "Did not update proper Linear Constraints"<<std::endl;
                std::cout << "================================="<<std::endl;
                std::cout << "================================="<<std::endl;
                return false;
            }
        }
        
        // std::cout<<"ee_pose: "<<std::endl << x_.translation() << std::endl << x_.rotation() << std::endl << "joint: " << std::endl << q_ << std::endl 
        // << "jacobian: " << std::endl << j_ << "desired_ee_pose: " << std::endl << desired_x_.translation() << std::endl << desired_x_.rotation() << std::endl 
        // << "obs_posi: " << std::endl << obs_position_ << "\n\n\n\n\n\n";

        // solve the QP problem
        if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
        if (solver_.getStatus() != OsqpEigen::Status::Solved) return false;

        // get the controller input
        auto QPSolution = solver_.getSolution();
        // std::cout<<QPSolution<<std::endl;
        opt_del_q_ = QPSolution.tail(dof);
        return true; 
        
        
    }

    Eigen::VectorXd QP::getJointDisplacement()
    {
        return opt_del_q_;
    }
    // -----------------------------------------------------------------------------------------

    // ------------------------------------ NN model -----------------------------------------------
    NNmodel::NNmodel(const std::string & file_path)
    {
        file_path_ = file_path;
    }

    NNmodel::~NNmodel()
    {
        std::cout<<"NN model terminate" <<std::endl;
    }

    void NNmodel::readWeightFile(int weight_num)
    {
        if (!mlp_.weight_files[weight_num].is_open())
        {
            std::cout << "Can not find the file: " << mlp_.w_path[weight_num] << std::endl;
        }
        for (int i = 0; i < mlp_.weight[weight_num].rows(); i++)
        {
            for (int j = 0; j < mlp_.weight[weight_num].cols(); j++)
            {
                mlp_.weight_files[weight_num] >> mlp_.weight[weight_num](i, j);
            }
        }
        mlp_.weight_files[weight_num].close();

        if (mlp_.loadweightfile_verbose == true)
        {
            std::cout << "weight_" << weight_num << ": \n"
                << mlp_.weight[weight_num] <<std::endl;
        }
    }

    void NNmodel::readBiasFile(int bias_num)
    {
        if (!mlp_.bias_files[bias_num].is_open())
        {
            std::cout << "Can not find the file: " << mlp_.b_path[bias_num] << std::endl;
        }
        for (int i = 0; i < mlp_.bias[bias_num].rows(); i++)
        {
            mlp_.bias_files[bias_num] >> mlp_.bias[bias_num](i);
        }
        mlp_.bias_files[bias_num].close();

        if (mlp_.loadbiasfile_verbose == true)
        {
            std::cout << "bias_" << bias_num - mlp_.n_layer << ": \n"
                << mlp_.bias[bias_num] << std::endl;
        }
    }

    void NNmodel::loadNetwork()
    {
        for (int i = 0; i < mlp_.n_layer; i++)
        {
            mlp_.w_path[i] = file_path_ + "weight_" + std::to_string(i) + ".txt";
            mlp_.b_path[i] = file_path_ + "bias_" + std::to_string(i) + ".txt";

            mlp_.weight_files[i].open(mlp_.w_path[i], std::ios::in);
            mlp_.bias_files[i].open(mlp_.b_path[i], std::ios::in);

            readWeightFile(i);
            readBiasFile(i);
        }
    }

    void NNmodel::initializeNetwork(int n_input, int n_output, Eigen::VectorXd n_hidden, bool is_nerf)
    {
        mlp_.is_nerf = is_nerf;
        mlp_.n_input = n_input;
        mlp_.n_output = n_output;
        mlp_.n_hidden = n_hidden;
        mlp_.n_layer = n_hidden.rows() + 1; // hiden layers + output layer

        mlp_.weight.resize(mlp_.n_layer);
        mlp_.bias.resize(mlp_.n_layer);
        mlp_.hidden.resize(mlp_.n_layer - 1);
        mlp_.hidden_derivative.resize(mlp_.n_layer - 1);

        mlp_.w_path.resize(mlp_.n_layer);
        mlp_.b_path.resize(mlp_.n_layer); 
        mlp_.weight_files.resize(mlp_.n_layer);
        mlp_.bias_files.resize(mlp_.n_layer); 

        //parameters resize
        for (int i = 0; i < mlp_.n_layer; i++)
        {
            if (i == 0)
            {
                if(mlp_.is_nerf) 
                {
                    mlp_.weight[i].setZero(mlp_.n_hidden(i), 3 * mlp_.n_input);
                    mlp_.hidden_derivative[i].setZero(mlp_.n_hidden(i), 3 * mlp_.n_input);
                }
                else
                {
                    mlp_.weight[i].setZero(mlp_.n_hidden(i), mlp_.n_input);
                    mlp_.hidden_derivative[i].setZero(mlp_.n_hidden(i), mlp_.n_input);
                }
                mlp_.bias[i].setZero(mlp_.n_hidden(i));
                mlp_.hidden[i].setZero(mlp_.n_hidden(i));
            }
            else if (i == mlp_.n_layer - 1)
            {
                mlp_.weight[i].setZero(mlp_.n_output, mlp_.n_hidden(i - 1));
                mlp_.bias[i].setZero(mlp_.n_output);
            }
            else
            {
                mlp_.weight[i].setZero(mlp_.n_hidden(i), mlp_.n_hidden(i - 1));
                mlp_.bias[i].setZero(mlp_.n_hidden(i));
                mlp_.hidden[i].setZero(mlp_.n_hidden(i));
                mlp_.hidden_derivative[i].setZero(mlp_.n_hidden(i), mlp_.n_hidden(i - 1));
            }
        }
        //input output resize
        mlp_.input.resize(mlp_.n_input);
        mlp_.input_nerf.resize(3 * mlp_.n_input);
        mlp_.output.resize(mlp_.n_output);
        mlp_.output_derivative.setZero(mlp_.n_output, mlp_.n_input);
    }

    void NNmodel::setNeuralNetwork(int n_input, int n_output, Eigen::VectorXd n_hidden, bool is_nerf)
    {
        // Eigen::VectorXd n_hidden;
        // n_hidden.resize(4);
        // n_hidden << 256, 256, 256, 256;
        initializeNetwork(n_input, n_output, n_hidden, is_nerf);
        loadNetwork();
    }

    std::pair<Eigen::VectorXd, Eigen::MatrixXd> NNmodel::calculateMlpOutput(Eigen::VectorXd input, bool time_verbose)
    {
        mlp_.input = input;
        if (mlp_.is_nerf)
        {
            Eigen::VectorXd sinInput = input.array().sin();
            Eigen::VectorXd cosInput = input.array().cos();

            mlp_.input_nerf.segment(0 * mlp_.n_input, mlp_.n_input) = input;
            mlp_.input_nerf.segment(1 * mlp_.n_input, mlp_.n_input) = sinInput;
            mlp_.input_nerf.segment(2 * mlp_.n_input, mlp_.n_input) = cosInput;
        }
        // std::cout<< "INPUT DATA:"<< std::endl <<mlp_.input.transpose() << std::endl;

        std::vector<clock_t> start, finish;
        start.resize(3*mlp_.n_layer);
        finish.resize(3*mlp_.n_layer);

        start[3*mlp_.n_layer - 1] = clock(); // Total 
        Eigen::MatrixXd temp_derivative;
        for (int layer = 0; layer < mlp_.n_layer; layer++)
        {
            if (layer == 0) // input layer
            {
                start[0] = clock(); // Linear 
                if (mlp_.is_nerf) mlp_.hidden[0] = mlp_.weight[0] * mlp_.input_nerf + mlp_.bias[0];
                else                mlp_.hidden[0] = mlp_.weight[0] * mlp_.input + mlp_.bias[0];
                finish[0] = clock();

                start[1] = clock(); // ReLU
                for (int h = 0; h < mlp_.n_hidden(layer); h++)
                {
                    mlp_.hidden_derivative[0].row(h) = ReLU_derivative(mlp_.hidden[0](h)) * mlp_.weight[0].row(h); //derivative wrt input
                    mlp_.hidden[0](h) = ReLU(mlp_.hidden[0](h));                                                     //activation function
                }
                finish[1] = clock();

                if (mlp_.is_nerf)
                {
                    Eigen::MatrixXd nerf_jac;
                    nerf_jac.setZero(3 * mlp_.n_input, mlp_.n_input);
                    nerf_jac.block(0 * mlp_.n_input, 0, mlp_.n_input, mlp_.n_input) = Eigen::MatrixXd::Identity(mlp_.n_input, mlp_.n_input);
                    nerf_jac.block(1 * mlp_.n_input, 0, mlp_.n_input, mlp_.n_input).diagonal() <<   mlp_.input.array().cos();
                    nerf_jac.block(2 * mlp_.n_input, 0, mlp_.n_input, mlp_.n_input).diagonal() << - mlp_.input.array().sin();
                    
                    start[2] = clock(); // Multip
                    temp_derivative = mlp_.hidden_derivative[0] * nerf_jac;
                    finish[2] = clock();
                }
                else
                {
                    temp_derivative = mlp_.hidden_derivative[0];
                }
            }
            else if (layer == mlp_.n_layer - 1) // output layer
            {
                start[layer*3] = clock(); // Linear
                mlp_.output = mlp_.weight[layer] * mlp_.hidden[layer - 1] + mlp_.bias[layer];
                finish[layer*3] = clock(); 

                start[layer*3+1] = clock(); // Multip
                mlp_.output_derivative = mlp_.weight[layer] * temp_derivative;
                finish[layer*3+1] = clock();
            }
            else // hidden layers
            {
                start[layer*3] = clock(); // Linear
                mlp_.hidden[layer] = mlp_.weight[layer] * mlp_.hidden[layer - 1] + mlp_.bias[layer];
                finish[layer*3] = clock();
                
                start[layer*3+1] = clock(); // ReLU
                for (int h = 0; h < mlp_.n_hidden(layer); h++)
                {
                    mlp_.hidden_derivative[layer].row(h) = ReLU_derivative(mlp_.hidden[layer](h)) * mlp_.weight[layer].row(h); //derivative wrt input
                    mlp_.hidden[layer](h) = ReLU(mlp_.hidden[layer](h));                                                         //activation function
                }
                finish[layer*3+1] = clock();

                start[3*layer+2] = clock(); //Multip
                temp_derivative = mlp_.hidden_derivative[layer] * temp_derivative;
                finish[3*layer+2] = clock();
            }
        }

        finish[3*mlp_.n_layer - 1] = clock();

        if(time_verbose)
        {
            std::cout<<"------------------Time[1e-6]------------------"<<std::endl;
            for (int layer = 0; layer < mlp_.n_layer; layer++)
            {
                if(layer == mlp_.n_layer - 1)
                {
                    std::cout<<"Layer "<<layer<<" -Linear: "<<double(finish[3*layer+0]-start[3*layer+0])<<std::endl;
                    std::cout<<"Layer "<<layer<<" -Multip: "<<double(finish[3*layer+1]-start[3*layer+1])<<std::endl;
                    std::cout<<"Total          : "          <<double(finish[3*layer+2]-start[3*layer+2])<<std::endl;
                }
                else
                {
                    std::cout<<"Layer "<<layer<<" -Linear: "<<double(finish[3*layer+0]-start[3*layer+0])<<std::endl;
                    std::cout<<"Layer "<<layer<<" -ReLU  : "<<double(finish[3*layer+1]-start[3*layer+1])<<std::endl;
                    std::cout<<"Layer "<<layer<<" -Multip: "<<double(finish[3*layer+2]-start[3*layer+2])<<std::endl;
                }
            }
            std::cout<<"------------------------------------------------"<<std::endl;
        }

        return std::make_pair(mlp_.output, mlp_.output_derivative); 
        // std::cout<< "OUTPUT DATA:"<< std::endl <<mlp_.output.transpose() << std::endl;
        // std::cout<< "OUTPUT DATA:"<< std::endl <<mlp_.output_derivative << std::endl;
    }
    // -----------------------------------------------------------------------------------------------
}