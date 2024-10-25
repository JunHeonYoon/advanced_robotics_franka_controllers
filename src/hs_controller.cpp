
#include <advanced_robotics_franka_controllers/hs_controller.h>
#include <cmath>
#include <memory>
#include <termios.h>
#include <fcntl.h>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include "math_type_define.h"

namespace advanced_robotics_franka_controllers
{
// ---------------------------default controller function-----------------------------------------
bool hs_controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
	std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "hs_controller: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("hs_controller: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "hs_controller: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("hs_controller: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "hs_controller: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("hs_controller: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "hs_controller: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface == nullptr) {
    ROS_ERROR_STREAM("hs_controller: Error getting position joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(position_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("hs_controller: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  q_desired_.setZero();
  qdot_desired_.setZero();
  torque_desired_.setZero();


  return true;
}

void hs_controller::starting(const ros::Time& time) {
  start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {
    q_(i) = joint_handles_[i].getPosition();
    qdot_(i) = joint_handles_[i].getVelocity();
  }
  q_init_ = q_;
  qdot_init_ = qdot_;
  
  q_goal_ << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4;

  elapsed_time_ = ros::Duration(0.0);
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  // transform_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());  
}


void hs_controller::update(const ros::Time& time, const ros::Duration& period) 
{
  elapsed_time_ += period;

  for(int i=0; i<7;i++)
  {
      q_desired_(i) = DyrosMath::cubic(elapsed_time_.toSec(), 0, 5.0, q_init_(i), q_goal_(i), 0, 0);
  }

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(q_desired_(i));
  }

}

// ------------------------------------------------------------------------------------------------


} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::hs_controller,
                       controller_interface::ControllerBase)
