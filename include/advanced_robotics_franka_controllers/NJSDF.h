#ifndef NJSDF_QP_H
#define NJSDF_QP_H

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>

#include "OsqpEigen/OsqpEigen.h"

#include "advanced_robotics_franka_controllers/common.h"

namespace NJSDF
{
    class NNmodel
    {
        struct MLP
        {
            ~MLP() { std::cout << "MLP terminate" << std::endl; }
            std::vector<Eigen::MatrixXd> weight;
            std::vector<Eigen::VectorXd> bias;
            std::vector<Eigen::VectorXd> hidden;
            std::vector<Eigen::MatrixXd> hidden_derivative;

            std::vector<std::string> w_path;
            std::vector<std::string> b_path;

            std::vector<std::ifstream> weight_files;
            std::vector<std::ifstream> bias_files;

            int n_input;
            int n_output;
            Eigen::VectorXd n_hidden;
            int n_layer;

            Eigen::VectorXd input;
            Eigen::VectorXd output;
            Eigen::MatrixXd output_derivative;

            bool is_nerf;
            Eigen::VectorXd input_nerf;

            bool loadweightfile_verbose = false;
            bool loadbiasfile_verbose = false;
        };
        
        public:
            NNmodel(const std::string & file_path = "/home/dyros/jh_ws/src/advanced_robotics_franka_controllers/NNmodel/parameter/");
            ~NNmodel();
            void setNeuralNetwork(int n_input, int n_output, Eigen::VectorXd n_hidden, bool is_nerf);
            std::pair<Eigen::VectorXd, Eigen::MatrixXd> calculateMlpOutput(Eigen::VectorXd input, bool time_verbose);

        private:
            std::string file_path_;
            MLP mlp_;

            void readWeightFile(int weight_num);
            void readBiasFile(int bias_num);
            void loadNetwork();
            void initializeNetwork(int n_input, int n_output, Eigen::VectorXd n_hidden, bool is_nerf);

            double ReLU(double input)
            {
                return std::max(0.0, input);
            }
            double ReLU_derivative(double input)
            {
                return (input > 0)? 1.0 : 0.0;
            }

    };

    class QP
    {
        enum barrier_func_type{LOG, LINEAR, RBF};

        public:
            QP(double obs_radius, int hz, barrier_func_type barrier_func = RBF);
            ~QP();
            void setCurrentState(Eigen::Affine3d ee_pose, Eigen::Matrix<double, dof, 1> joint, Eigen::Matrix<double, ee_dof, dof> jacobian);
            void setDesiredState(Eigen::Affine3d desired_ee_pose);
            void setObsPosition(Eigen::Matrix<double,3,1> obs_position);
            bool solveQP(bool verbose=false);
            Eigen::VectorXd getJointDisplacement();

        private:
            double obs_radius_;
            double hz_;
            barrier_func_type barrier_func_;
            
            std::shared_ptr<NNmodel> njsdf_model_;

            Eigen::Matrix<double, dof, 1>         joint_lower_lim_, joint_upper_lim_;
            Eigen::Matrix<double, ee_dof, 1>      ee_vel_lower_lim_, ee_vel_upper_lim_;
            Eigen::Matrix<double, ee_dof, ee_dof> slack_weight_;
            Eigen::Matrix<double, dof, dof>       damping_weight_;

            Eigen::Affine3d x_, desired_x_; // rotation + position
            Eigen::Matrix<double, dof, 1> q_, opt_del_q_; // joint angle
            Eigen::Matrix<double, ee_dof, dof> j_; // jacobian
            Eigen::Matrix<double, 3, 1>  obs_position_;

            Eigen::Matrix<double, num_links, 1> pred_min_dist_;
            Eigen::Matrix<double, num_links, dof> pred_min_dist_grad_;

            bool is_first_;

            OsqpEigen::Solver solver_;

            void setJointLimit(Eigen::Matrix<double, dof, 1> lower_lim, Eigen::Matrix<double, dof, 1> upper_lim);
            void setEEVelocityLimit(Eigen::Matrix<double, ee_dof, 1> lower_lim, Eigen::Matrix<double, ee_dof, 1> upper_lim);
            void setWeightMatrix(Eigen::Matrix<double, ee_dof, ee_dof> slack_weight, Eigen::Matrix<double, dof, dof> damping_weight);
            std::pair<Eigen::Matrix<double, num_links, 1>, Eigen::Matrix<double, num_links, dof>> getSD();
            
            Eigen::Matrix<double, num_links, 1> RBFunc(double delta, double mu, Eigen::Matrix<double, num_links, 1> h)
            {
                // Chiu, J. R., Sleiman, J. P., Mittal, M., Farshidian, F., & Hutter, M. (2022, May). 
                // A collision-free mpc for whole-body dynamic locomotion and manipulation. 
                // In 2022 International Conference on Robotics and Automation (ICRA) (pp. 4686-4693). IEEE.
                Eigen::Matrix<double, num_links, 1> result;
                for (size_t i=0; i<result.size(); i++)
                {
                    if (h[i] >= delta)
                    {
                        result[i] = -mu * log(h[i]);
                    }
                    else
                    {
                        result[i] = mu * ( pow(h[i],2) / (2*pow(delta,2)) - 2*h[i] / delta + 1.5 - log(delta) );
                    }
                }
                return result;
            }
    };

}

#endif  // NJSDF_QP_H
