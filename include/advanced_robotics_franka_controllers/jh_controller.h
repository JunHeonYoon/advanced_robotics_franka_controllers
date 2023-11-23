
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <thread>
#include <mutex>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/trigger_rate.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>

#include "advanced_robotics_franka_controllers/NJSDF.h"
#include "advanced_robotics_franka_controllers/robot.h"
#include <suhan_benchmark.h>

namespace advanced_robotics_franka_controllers {

class jh_controller : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
								   hardware_interface::EffortJointInterface,
								   franka_hw::FrankaStateInterface> {
                     
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time);

 private: 
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  franka_hw::TriggerRate print_rate_trigger_{10}; 

  std::shared_ptr<NJSDF::RobotModel> robot_;
	
  // initial state
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Matrix<double, 7, 1> qdot_init_;

  // current state
  Eigen::Matrix<double, 7, 1> q_;
  Eigen::Matrix<double, 7, 1> qdot_;
  Eigen::Matrix<double, 7, 1> torque_;

  // control value
  Eigen::Matrix<double, 7, 1> q_desired_;
  Eigen::Matrix<double, 7, 1> qdot_desired_;
  Eigen::Matrix<double, 7, 1> torque_desired_;


  // Task space
	Eigen::Matrix<double, 3, 1> x_;
	Eigen::Matrix<double, 3, 1> x_init_;
	Eigen::Matrix<double, 3, 3> rotation_;
	Eigen::Matrix<double, 3, 3> rotation_init_;
	Eigen::Matrix<double, 3, 1> phi_;
	Eigen::Matrix<double, 6, 1> x_dot_;   // 6D (linear + angular)
	Eigen::Matrix<double, 6, 1> x_error_;

  // dynamics
  Eigen::Matrix<double, 7, 1> g_; // gravity matrix
  Eigen::Matrix<double, 7, 7> m_; // mass matrix
  Eigen::Matrix<double, 7, 7> m_inv_; // Inverse of mass matrix
  Eigen::Matrix<double, 7, 1> c_; // coliolis matrix

  // For controller
	Eigen::Matrix<double, 6, 7> j_;	// Full basic Jacobian matrix
	Eigen::Matrix<double, 3, 7> j_v_;	// Linear velocity Jacobian matrix
	Eigen::Matrix<double, 3, 7> j_w_;	// Angular veolicty Jacobain matrix

  // transformation matrix
  Eigen::Affine3d transform_init_;
  Eigen::Affine3d transform_;

  const double hz_{1000};
  ros::Time start_time_;
  ros::Time play_time_;
  ros::Time control_start_time_;
  SuhanBenchmark bench_timer_;

  enum CTRL_MODE{NONE, HOME, NJSDF};
  CTRL_MODE control_mode_{NONE};
	bool is_mode_changed_ {false};


  std::shared_ptr<NJSDF::QP> njsdf_qp_;
  const double njsdf_hz_{100};
  franka_hw::TriggerRate njsdf_trigger_{njsdf_hz_}; 
  double obs_radius_{0.10};
  Eigen::Vector3d obs_position_;
  Eigen::Affine3d target_ee_pose_;
  Eigen::Vector3d target_ee_vel_;
  Eigen::Matrix<double, 7, 1> opt_dq_;

  
  std::mutex calculation_mutex_;
  std::mutex njsdf_input_mutex_;
  std::mutex njsdf_mutex_;

  bool quit_all_proc_{false};
  std::thread async_calculation_thread_;
  std::thread async_njsdf_thread_;
  std::thread mode_change_thread_;
  bool njsdf_thread_enabled_{false};

  void printState();
  void moveJointPositionTorque(const Eigen::Matrix<double, 7, 1> & target_q, double duration);
  
  void setMode(const CTRL_MODE & mode);
  void getCurrentState();
  void setDesiredTorque(const Eigen::Matrix<double, 7, 1> & desired_torque);

  void modeChangeReaderProc();
  void asyncCalculationProc();
  void asyncNJSDFProc();
};

}  // namespace advanced_robotics_franka_controllers
