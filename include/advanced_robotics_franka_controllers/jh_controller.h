
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

#include "suhan_benchmark.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>

// ============== MPCC ==============
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "MPC/mpc.h"
#include "Params/track.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/JointState.h>
// ==================================

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
	Eigen::Matrix<double, 6, 1> wrench_;


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

  enum CTRL_MODE{NONE, HOME, MPCC};
  CTRL_MODE control_mode_{NONE};
	bool is_mode_changed_ {false};

  std::mutex calculation_mutex_;
  std::mutex input_mutex_;

  bool quit_all_proc_{false};
  std::thread async_calculation_thread_;
  std::thread mode_change_thread_;

  actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_ac_
  {"/franka_gripper/move", true};

  void printState();
  Eigen::Matrix<double, 7, 1> PDControl(const Eigen::Matrix<double, 7, 1> & q_desired, const Eigen::Matrix<double, 7, 1> & qdot_desired);
  void moveJointPositionTorque(const Eigen::Matrix<double, 7, 1> & target_q, double duration);
  
  void setMode(const CTRL_MODE & mode);
  void getCurrentState();
  void setDesiredTorque(const Eigen::Matrix<double, 7, 1> & desired_torque);

  void modeChangeReaderProc();
  void asyncCalculationProc();

  // ============== MPCC ==============
  struct PathParmeter
  {
    double s;
    double vs;
    double dVs;
    void setZero(){s=0; vs=0; dVs=0;}
  };

  std::unique_ptr<mpcc::Integrator> integrator_;
  std::unique_ptr<mpcc::MPC> mpc_;
  mpcc::PathToJson json_paths_;
  franka_hw::TriggerRate mpcc_trigger_;

  double Ts_mpcc_;
  double hz_mpcc_;
  Eigen::Matrix<double, 7, 1> mpcc_qdot_desired_;
  double mpcc_dVs_desired_;
  PathParmeter s_info_;
  bool is_mpcc_solved_{false};

  std::thread async_mpcc_thread_;
  bool mpcc_thread_enabled_{false};
  std::mutex mpcc_mutex_;
  std::mutex mpcc_input_mutex_;

  void asyncMPCCFProc();

  ros::Publisher mpcc_global_path_pub_;
  ros::Publisher mpcc_local_path_pub_;
  ros::Publisher mpcc_ref_path_pub_;
  ros::Publisher ee_pose_pub_;
  

  bool is_contacted_{false};
  bool is_contacted_changed_{false};
  ros::Time contact_time_;
  Eigen::Matrix<double,7,1> Kp_diag_, Kv_diag_;
  Eigen::Matrix<double,7,1> Kp_diag_init_, Kv_diag_init_;
  double contact_thres{6.5};
  // ==================================
};

}  // namespace advanced_robotics_franka_controllers
