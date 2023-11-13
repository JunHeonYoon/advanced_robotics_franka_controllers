
#include <advanced_robotics_franka_controllers/jh_controller.h>
#include <cmath>
#include <memory>
#include <termios.h>
#include <fcntl.h>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include "math_type_define.h"

int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF)
	{
	ungetc(ch, stdin);
	return 1;
	}

	return 0;
}

namespace advanced_robotics_franka_controllers
{
// ---------------------------default controller function-----------------------------------------
bool jh_controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
	std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "ForceExampleController: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  return true;
}

void jh_controller::starting(const ros::Time& time) {
  start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {
    q_init_(i) = joint_handles_[i].getPosition();
    qdot_init_(i) = joint_handles_[i].getVelocity();
  }
  
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());  
  x_init_ = transform_init_.translation();
  rotation_init_ = transform_init_.rotation();
}

void jh_controller::update(const ros::Time& time, const ros::Duration& period) 
{
 if(kbhit())
 {
  int key = getchar();
  switch (key)
  {
    case 'h':
      jh_controller::setMode("joint_ctrl_home");
      break;
    default:
      break;
  }
 } 
 jh_controller::compute();
 play_time_ = time;
 jh_controller::setDesiredTorque(torque_desired_);
}
// ------------------------------------------------------------------------------------------------

// --------------------------- funciotn from robotics class -----------------------------------------
void jh_controller::compute()
{
  jh_controller::getCurrentState(); // compute q(dot), dynamic, jacobian, EE pose(velocity)

  if(is_mode_changed_)
  {
    is_mode_changed_ = false;
    control_start_time_ = play_time_;
    q_init_ = q_;
    qdot_init_ = qdot_;
    x_init_ = x_;
    rotation_init_ = rotation_;
  }

  if(control_mode_ == "joint_ctrl_home")
  {
    Eigen::Matrix<double, 7, 1> target_q;
    target_q << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
    jh_controller::moveJointPositionTorque(target_q, 5.0);
  }
  else
  {
    torque_desired_ = c_;
  }

  jh_controller::printState();
}

void jh_controller::printState()
{
  if (print_rate_trigger_()) 
    {
    std::cout << "time     : " << std::fixed << std::setprecision(3) << play_time_.toSec() << std::endl;
		std::cout << "q now    :\t";
		std::cout << std::fixed << std::setprecision(3) << q_.transpose() << std::endl;
		std::cout << "q desired:\t";
		std::cout << std::fixed << std::setprecision(3) << q_desired_.transpose() << std::endl;
		std::cout << "x        :\t";
		std::cout << x_.transpose() << std::endl;
		std::cout << "R        :\t" << std::endl;
		std::cout << std::fixed << std::setprecision(3) << rotation_ << std::endl;
    std::cout << "torque desired    :\t";
		std::cout << std::fixed << std::setprecision(3) << torque_desired_.transpose() << std::endl;
  }
}

void jh_controller::moveJointPositionTorque(const Eigen::Matrix<double, 7, 1> &target_q, double duration)
{
  for(size_t i=0; i<7;i++)
  {
    q_desired_(i) = DyrosMath::cubic(play_time_.toSec(), control_start_time_.toSec(), control_start_time_.toSec() + duration,
                                        q_init_(i), target_q(i), 0, 0);
    qdot_desired_(i) = DyrosMath::cubicDot(play_time_.toSec(), control_start_time_.toSec(), control_start_time_.toSec() + duration,
                                        q_init_(i), target_q(i), 0, 0);
  }

  double kp, kv;
  kp = 1500;
  kv = 10;

  torque_desired_ = m_ * ( kp*(q_desired_ - q_) + kv*(qdot_desired_ - qdot_)) + c_;
}

// --------------------------- Controller Core Methods -----------------------------------------
void jh_controller::setMode(const std::string & mode)
{
  is_mode_changed_ = true;
  control_mode_ = mode;
  std::cout << "Current mode (changed): " << mode << std::endl;
}

void jh_controller::getCurrentState()
{
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  const std::array<double, 42> &jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  const std::array<double, 7> &gravity_array = model_handle_->getGravity();
  const std::array<double, 49> &massmatrix_array = model_handle_->getMass();
  const std::array<double, 7> &coriolis_array = model_handle_->getCoriolis();


  q_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
  qdot_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());
  torque_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.tau_J.data());
  g_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(gravity_array.data());
  m_ = Eigen::Map<const Eigen::Matrix<double, 7, 7>>(massmatrix_array.data());
  m_inv_ = m_.inverse();
  c_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
  j_ = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian_array.data());
  j_v_ = j_.block<3, 7>(0, 0);
  j_w_ = j_.block<3, 7>(3, 0);
  transform_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());  
  x_ = transform_.translation();
  rotation_ = transform_.rotation();
  x_dot_ = j_ * qdot_;
}

void jh_controller::setDesiredTorque(const Eigen::Matrix<double, 7, 1> & desired_torque)
{
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(desired_torque(i));
  }
}
// ------------------------------------------------------------------------------------------------




} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::jh_controller,
                       controller_interface::ControllerBase)
