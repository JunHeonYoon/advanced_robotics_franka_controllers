#ifndef JH_CONTROLLER_H
#define JH_CONTROLLER_H

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
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Int8MultiArray.h"
#include <Eigen/Dense>

#include "advanced_robotics_franka_controllers/QP_controller.h"
#include "suhan_benchmark.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>


namespace advanced_robotics_franka_controllers {

class jh_controller : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
                  //  hardware_interface::PositionJointInterface,       
                   hardware_interface::VelocityJointInterface,
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
  franka_hw::TriggerRate qp_controller_trigger_{1000}; 
	
  // initial state
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Matrix<double, 7, 1> qdot_init_;

  // current state
  Eigen::Matrix<double, 7, 1> q_;
  Eigen::Matrix<double, 7, 1> qdot_;
  // Eigen::Matrix<double, 7, 1> torque_;

  // control value
  Eigen::Matrix<double, 7, 1> q_desired_;
  Eigen::Matrix<double, 7, 1> qdot_desired_;


  // Task space
	Eigen::Matrix<double, 3, 1> x_;
	Eigen::Matrix<double, 3, 1> x_init_;
	Eigen::Matrix<double, 3, 3> rotation_;
	Eigen::Matrix<double, 3, 3> rotation_init_;
	Eigen::Matrix<double, 3, 1> phi_;
	Eigen::Matrix<double, 6, 1> x_dot_;   // 6D (linear + angular)
	Eigen::Matrix<double, 6, 1> x_error_;

  // dynamics
  // Eigen::Matrix<double, 7, 1> g_; // gravity matrix
  // Eigen::Matrix<double, 7, 7> m_; // mass matrix
  // Eigen::Matrix<double, 7, 7> m_inv_; // Inverse of mass matrix
  // Eigen::Matrix<double, 7, 1> c_; // coliolis matrix

  // For controller
	Eigen::Matrix<double, 6, 7> j_;	// Full basic Jacobian matrix
	Eigen::Matrix<double, 3, 7> j_v_;	// Linear velocity Jacobian matrix
	Eigen::Matrix<double, 3, 7> j_w_;	// Angular veolicty Jacobain matrix

  // transformation matrix
  Eigen::Affine3d transform_init_;
  Eigen::Affine3d transform_;

  const double hz_{1000.};
  ros::Time start_time_;
  ros::Time play_time_;
  ros::Time control_start_time_;
  SuhanBenchmark bench_timer_;

  enum CTRL_MODE{NONE, HOME, TELEOPERATE};
  CTRL_MODE control_mode_{NONE};
	bool is_mode_changed_ {false};

  std::unique_ptr<QP_CONTROLLER::QP> qp_controller_;

  std::mutex calculation_mutex_;
  std::mutex qp_controller_input_mutex_;

  bool quit_all_proc_{false};
  std::thread async_calculation_thread_;
  std::thread async_qp_controller_thread_;
  std::thread mode_change_thread_;
  bool qp_controller_thread_enabled_ = false;
  bool tmp_use = false;

  ros::Subscriber haptic_pose_sub_;
  ros::Subscriber haptic_twist_sub_;
  ros::Subscriber haptic_button_sub_;
  Eigen::Matrix<double, 6, 1> haptic_vel_command_; // Linear and Angular velocity
  
  int pre_button_state{0};

  enum GRIPPER_MODE{OPEN, CLOSE};
  GRIPPER_MODE gripper_command_{OPEN};
  actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_ac_close_
  {"/franka_gripper/grasp", true};
  actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_ac_open_
  {"/franka_gripper/move", true};
  actionlib::SimpleActionClient<franka_gripper::HomingAction> gripper_ac_homing_
  {"/franka_gripper/homing", true};

  void hapticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void hapticTwistCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void hapticButtonCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);

  Eigen::MatrixXd LowPassFilter(const Eigen::MatrixXd &input, const Eigen::MatrixXd &prev_res, const double &sampling_freq, const double &cutoff_freq);

  void printState();
  void moveJointPosition(const Eigen::Matrix<double, 7, 1> & target_q, double duration);
  
  void setMode(const CTRL_MODE & mode);
  void getCurrentState();
  void setDesiredJointVel(const Eigen::Matrix<double, 7, 1> & desired_qdot);

  void modeChangeReaderProc();
  void asyncCalculationProc();
  void asyncQPControllerProc();
};

}  // namespace advanced_robotics_franka_controllers

#endif