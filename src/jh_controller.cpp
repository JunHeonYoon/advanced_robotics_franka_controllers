
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

  mode_change_thread_ = std::thread(&jh_controller::modeChangeReaderProc, this);
  q_desired_.setZero();
  qdot_desired_.setZero();
  torque_desired_.setZero();


  return true;
}

void jh_controller::starting(const ros::Time& time) {
  start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {
    q_(i) = joint_handles_[i].getPosition();
    qdot_(i) = joint_handles_[i].getVelocity();
  }
  q_init_ = q_;
  qdot_init_ = qdot_;
  
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());  


  x_ = transform_.translation();
  rotation_ = transform_.rotation();
  transform_init_ = transform_;
  x_init_ = x_;
  rotation_init_ = rotation_;
}

void jh_controller::update(const ros::Time& time, const ros::Duration& period) 
{

  jh_controller::getCurrentState(); // compute q(dot), dynamic, jacobian, EE pose(velocity)

  if(calculation_mutex_.try_lock())
  {
      calculation_mutex_.unlock();
      if(async_calculation_thread_.joinable()) async_calculation_thread_.join();
      async_calculation_thread_ = std::thread(&jh_controller::asyncCalculationProc, this);
  }
  ros::Rate r(30000);
  for(size_t i=0; i<9; ++i)
  {
      r.sleep();
      if(calculation_mutex_.try_lock())
      {
          calculation_mutex_.unlock();
          if(async_calculation_thread_.joinable()) async_calculation_thread_.join();
          break;
      }
  }

  jh_controller::printState();
  play_time_ = time;
  jh_controller::setDesiredTorque(torque_desired_);
}

void jh_controller::stopping(const ros::Time & /*time*/)
{
  ROS_INFO("jh_controller::stopping");
}
// ------------------------------------------------------------------------------------------------

// --------------------------- funciotn from robotics class -----------------------------------------
void jh_controller::printState()
{
  if (print_rate_trigger_()) 
    {
    std::cout << "-------------------------------------------------------------------" << std::endl;
    std::cout << "MODE     : " << control_mode_ << std::endl;
    std::cout << "time     : " << std::fixed << std::setprecision(3) << play_time_.toSec() << std::endl;
		std::cout << "q now    :\t";
		std::cout << std::fixed << std::setprecision(3) << q_.transpose() << std::endl;
		std::cout << "q desired:\t";
		std::cout << std::fixed << std::setprecision(3) << q_desired_.transpose() << std::endl;
		std::cout << "x        :\t";
		std::cout << x_.transpose() << std::endl;
		std::cout << "R        :\t" << std::endl;
		std::cout << std::fixed << std::setprecision(3) << rotation_ << std::endl;
    std::cout << "J        :\t" << std::endl;
		std::cout << std::fixed << std::setprecision(3) << j_ << std::endl;
    std::cout << "torque desired    :\t";
		std::cout << std::fixed << std::setprecision(3) << torque_desired_.transpose() << std::endl;
    std::cout << "-------------------------------------------------------------------\n\n" << std::endl;
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
void jh_controller::setMode(const CTRL_MODE & mode)
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

void jh_controller::asyncCalculationProc()
  {
    bench_timer_.reset();
    calculation_mutex_.lock();
    if(is_mode_changed_)
    {
      is_mode_changed_ = false;
      control_start_time_ = play_time_;
      q_init_ = q_;
      qdot_init_ = qdot_;
      x_init_ = x_;
      rotation_init_ = rotation_;
      transform_init_ = transform_;
    }

    if(control_mode_ == HOME)
    {
      Eigen::Matrix<double, 7, 1> target_q;
      target_q << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
      target_q << 0.516,  0.447,  0.384, -1.085, -0.143, 1.587,  1.517;
      jh_controller::moveJointPositionTorque(target_q, 5.0);
    }
    else
    {
      torque_desired_ = c_;
    }
    calculation_mutex_.unlock();
    double elapsed_time = bench_timer_.elapsedAndReset();
    // std::cout << "elapsed_time: " << elapsed_time*1000.0 << std::endl;
  }

void jh_controller::modeChangeReaderProc()
{
   while (!quit_all_proc_)
    {
      if(kbhit())
      {
        calculation_mutex_.lock();
        int key = getchar();
        switch (key)
        {
          case 'h':
            jh_controller::setMode(HOME);
            break;
          default:
            jh_controller::setMode(NONE);
            break;
        }
        calculation_mutex_.unlock();
      }
      
    }
}
// ------------------------------------------------------------------------------------------------




} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::jh_controller,
                       controller_interface::ControllerBase)
