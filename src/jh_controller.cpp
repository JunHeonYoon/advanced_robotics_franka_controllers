
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
      "jh_controller: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("jh_controller: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "jh_controller: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("jh_controller: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "jh_controller: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("jh_controller: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "jh_controller: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* velocity_joint_interface = robot_hw->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface == nullptr) {
    ROS_ERROR_STREAM("jh_controller: Error getting velocity joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(velocity_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("jh_controller: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  mode_change_thread_ = std::thread(&jh_controller::modeChangeReaderProc, this);
  q_desired_.setZero();
  qdot_desired_.setZero();

  qp_controller_ = std::make_unique<QP_CONTROLLER::QP>();
  async_qp_controller_thread_ = std::thread(&jh_controller::asyncQPControllerProc, this);

  haptic_pose_sub_ = node_handle.subscribe<geometry_msgs::PoseStamped>("/haptic/pose", 1, &jh_controller::hapticPoseCallback, this);
  haptic_twist_sub_ = node_handle.subscribe<geometry_msgs::Twist>("/haptic/twist", 1, &jh_controller::hapticTwistCallback, this);
  haptic_button_sub_ = node_handle.subscribe<std_msgs::Int8MultiArray>("/haptic/button_state", 1, &jh_controller::hapticButtonCallback, this);
  haptic_vel_command_.setZero();

  gripper_ac_homing_.waitForServer();  
  gripper_ac_homing_.sendGoal(franka_gripper::HomingGoal());

  return true;
}

void jh_controller::starting(const ros::Time& time) {
  start_time_ = time;
  play_time_ = time;
  control_start_time_ = time;

	
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

  play_time_ += period;
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
  jh_controller::setDesiredJointVel(qdot_desired_);
  tmp_use = true;
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
    std::cout << "qdot now:\t";
		std::cout << std::fixed << std::setprecision(3) << qdot_.transpose() << std::endl;
    std::cout << "qdot desired:\t";
		std::cout << std::fixed << std::setprecision(3) << qdot_desired_.transpose() << std::endl;
		std::cout << "x        :\t";
		std::cout << x_.transpose() << std::endl;
		std::cout << "R        :\t" << std::endl;
		std::cout << std::fixed << std::setprecision(3) << rotation_ << std::endl;
    std::cout << "J        :\t" << std::endl;
		std::cout << std::fixed << std::setprecision(3) << j_ << std::endl;
    std::cout << "haptic command :\t";
		std::cout << std::fixed << std::setprecision(3) << haptic_vel_command_.transpose() << std::endl;
    std::cout << "gripper mode   :\t";
		if(gripper_command_ == OPEN) std::cout << "OPEN" << std::endl;
		else if(gripper_command_ == CLOSE) std::cout << "CLOSE" << std::endl;

    std::cout << "-------------------------------------------------------------------\n\n" << std::endl;
  }
}

void jh_controller::moveJointPosition(const Eigen::Matrix<double, 7, 1> &target_q, double duration)
{
  for(size_t i=0; i<7;i++)
  {
    q_desired_(i) = DyrosMath::cubic(play_time_.toSec(), control_start_time_.toSec(), control_start_time_.toSec() + duration,
                                        q_init_(i), target_q(i), 0, 0);
    qdot_desired_(i) = DyrosMath::cubicDot(play_time_.toSec(), control_start_time_.toSec(), control_start_time_.toSec() + duration,
                                        q_init_(i), target_q(i), 0, 0);
  }
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
  // const std::array<double, 7> &gravity_array = model_handle_->getGravity();
  // const std::array<double, 49> &massmatrix_array = model_handle_->getMass();
  // const std::array<double, 7> &coriolis_array = model_handle_->getCoriolis();


  // q_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
  // qdot_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());
  for (size_t i = 0; i < 7; ++i) {
    q_(i) = joint_handles_[i].getPosition();
    qdot_(i) = joint_handles_[i].getVelocity();
  }
  // torque_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.tau_J.data());
  // g_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(gravity_array.data());
  // m_ = Eigen::Map<const Eigen::Matrix<double, 7, 7>>(massmatrix_array.data());
  // m_inv_ = m_.inverse();
  // c_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
  j_ = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian_array.data());
  j_v_ = j_.block<3, 7>(0, 0);
  j_w_ = j_.block<3, 7>(3, 0);
  transform_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());  
  x_ = transform_.translation();
  rotation_ = transform_.rotation();
  x_dot_ = j_ * qdot_;
}

void jh_controller::setDesiredJointVel(const Eigen::Matrix<double, 7, 1> & desired_qdot)
{
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(desired_qdot(i));
  }
}

void jh_controller::asyncQPControllerProc()
{
  SuhanBenchmark timer;
  while(!quit_all_proc_)
  {
    if(qp_controller_thread_enabled_)
    {
      timer.reset();
      qp_controller_input_mutex_.lock();
      qp_controller_->setCurrentState(q_, qdot_, j_);
      qp_controller_->setDesiredEEVel(haptic_vel_command_);
      qp_controller_input_mutex_.unlock();

      Eigen::Matrix<double, 7, 1> opt_qdot;
      QP_CONTROLLER::TimeDuration time_status;
      bool status = qp_controller_->solveQP(opt_qdot, time_status);
      if(status)
      {
        // ROS_INFO("QP solved!!!");
        qp_controller_input_mutex_.lock();
        qdot_desired_ = opt_qdot;
        qp_controller_input_mutex_.unlock();
      }
      else
      {
        opt_qdot.setZero();
        ROS_INFO("QP did not solved!!!");
        qdot_desired_ = opt_qdot;
      }
      double elapsed_time = timer.elapsedAndReset();
      if(print_rate_trigger_())
      {
        std::cout << "qp controller hz: " << 1. / elapsed_time << std::endl;
        std::cout << "qp set_qp  hz   : " << 1. / time_status.set_qp << std::endl;
        std::cout << "qp set_solver hz: " << 1. / time_status.set_solver << std::endl;
        std::cout << "qp solve_qp hz  : " << 1. / time_status.solve_qp << std::endl;

      }
    }
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
      q_desired_ = q_init_;
      qdot_desired_ = qdot_init_;
      x_init_ = x_;
      rotation_init_ = rotation_;
      transform_init_ = transform_;
      qp_controller_input_mutex_.lock();
      if(control_mode_ == TELEOPERATE) 
      {
        qp_controller_thread_enabled_ = true;
      }
      else qp_controller_thread_enabled_ = false;
      qp_controller_input_mutex_.unlock();
    }

    if(control_mode_ == HOME)
    {
      Eigen::Matrix<double, 7, 1> target_q;
      target_q << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
      jh_controller::moveJointPosition(target_q, 5.0);
    }
    else if(control_mode_ == TELEOPERATE)
    {
      if(tmp_use)
      {
        tmp_use = false;
        // asyncQPControllerProc();
      }
      q_desired_ = q_ + qdot_desired_ / hz_;
      

    }
    else
    {
      for(size_t i=0; i<7;i++)
      {
        qdot_desired_(i) = DyrosMath::cubic(play_time_.toSec(), control_start_time_.toSec(), control_start_time_.toSec() + 3.0,
                                            qdot_init_(i), 0.0, 0, 0);
      }
      q_desired_ = q_ + qdot_desired_ / hz_;
    }
    calculation_mutex_.unlock();
    double elapsed_time = bench_timer_.elapsedAndReset();
    if(print_rate_trigger_()) std::cout << "calculation proc freq: " << 1./elapsed_time << std::endl;
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
          case 't':
            jh_controller::setMode(TELEOPERATE);
            break;
          default:
            jh_controller::setMode(NONE);
            break;
        }
        calculation_mutex_.unlock();
      }
      
    }
}

void jh_controller::hapticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  double max_lin_vel = 0.1;

    Eigen::Vector3d lin_command;
    lin_command.setZero();
    if(fabs(msg->pose.position.x) > 0.01) lin_command(0) = std::min(max_lin_vel, std::max(-max_lin_vel, -msg->pose.position.x));
    if(fabs(msg->pose.position.y) > 0.01) lin_command(1) = std::min(max_lin_vel, std::max(-max_lin_vel, -msg->pose.position.y));
    if(fabs(msg->pose.position.z) > 0.01) lin_command(2) = std::min(max_lin_vel, std::max(-max_lin_vel, msg->pose.position.z));

    haptic_vel_command_.head(3) = lin_command;
    // haptic_vel_command_.head(3) = LowPassFilter(lin_command, haptic_vel_command_.head(3), 1000.0, 1.0);
}

void jh_controller::hapticTwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double max_ang_vel = 0.3;

    Eigen::Vector3d ang_command;
    ang_command.setZero();
    // if(fabs(msg->angular.x) > 0.0) ang_command(0) = std::min(max_ang_vel, std::max(-max_ang_vel, msg->angular.x));
    // if(fabs(msg->angular.y) > 0.0) ang_command(1) = std::min(max_ang_vel, std::max(-max_ang_vel, msg->angular.y));
    if(fabs(msg->angular.z) > 0.0) ang_command(2) = std::min(max_ang_vel, std::max(-max_ang_vel, msg->angular.z));

    haptic_vel_command_.tail(3) = ang_command;
    haptic_vel_command_.tail(3) = LowPassFilter(ang_command, haptic_vel_command_.tail(3), 1000.0, 1.0);
}

void jh_controller::hapticButtonCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
  if(pre_button_state == 0)
  {
    if(msg->data[0] == 1)
    {
      if(gripper_command_ == OPEN)
      {
        gripper_ac_close_.waitForServer();  
        franka_gripper::GraspGoal goal;
        goal.speed = 0.1;
        goal.force = 0.01;
        goal.epsilon.inner = 0.001;
        goal.epsilon.outer = 7.;
        gripper_ac_close_.sendGoal(goal);
        gripper_command_ = CLOSE; 
      }
      else if(gripper_command_ == CLOSE)
      {
        gripper_ac_open_.waitForServer();  
        franka_gripper::MoveGoal goal;
        goal.speed = 0.1;
        goal.width = 0.08;
        gripper_ac_open_.sendGoal(goal);
        gripper_command_ = OPEN;
      }
    }
  }
  pre_button_state = msg->data[0];
}

Eigen::MatrixXd jh_controller::LowPassFilter(const Eigen::MatrixXd &input, const Eigen::MatrixXd &prev_res, const double &sampling_freq, const double &cutoff_freq)
{

  double rc = 1. / (cutoff_freq * 2 * M_PI);
  double dt = 1. / sampling_freq;
  double a = dt / (rc + dt);
  return prev_res + a * (input - prev_res);
}


// ------------------------------------------------------------------------------------------------




} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::jh_controller,
                       controller_interface::ControllerBase)
