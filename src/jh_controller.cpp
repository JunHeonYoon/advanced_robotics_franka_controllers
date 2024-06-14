
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

#include <franka_gripper/GraspAction.h>

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

void rotationMatrixToQuaternion(const Eigen::Matrix<double, 3, 3>& rotation_matrix, geometry_msgs::Pose& pose_msg) {
    // Convert the Eigen rotation matrix to a tf2::Matrix3x3
    tf2::Matrix3x3 tf2_matrix(
        rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
        rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
        rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2)
    );

    // Convert the tf2::Matrix3x3 to a tf2::Quaternion
    tf2::Quaternion tf2_quaternion;
    tf2_matrix.getRotation(tf2_quaternion);

    // Convert the tf2::Quaternion to a geometry_msgs::Quaternion
    tf2::convert(tf2_quaternion, pose_msg.orientation);
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
  async_mpcc_thread_ = std::thread(&jh_controller::asyncMPCCFProc, this);
  q_desired_.setZero();
  qdot_desired_.setZero();
  torque_desired_.setZero();

  gripper_ac_.waitForServer();

  // ============== MPCC ==============
  std::ifstream iConfig(mpcc::pkg_path + "Params/config.json");
  nlohmann::json jsonConfig;
  iConfig >> jsonConfig;
  json_paths_ = {mpcc::pkg_path + std::string(jsonConfig["model_path"]),
                  mpcc::pkg_path + std::string(jsonConfig["cost_path"]),
                  mpcc::pkg_path + std::string(jsonConfig["bounds_path"]),
                  mpcc::pkg_path + std::string(jsonConfig["track_path"]),
                  mpcc::pkg_path + std::string(jsonConfig["normalization_path"]),
                  mpcc::pkg_path + std::string(jsonConfig["sqp_path"])};

  Ts_mpcc_ = jsonConfig["Ts"];
  hz_mpcc_ = 1./Ts_mpcc_;
  mpc_ = std::make_unique<mpcc::MPC>(jsonConfig["Ts"],json_paths_);
  mpcc_trigger_ = franka_hw::TriggerRate(hz_mpcc_);
  integrator_ = make_unique<mpcc::Integrator>(Ts_mpcc_, json_paths_);

  mpcc_global_path_pub_ = node_handle.advertise<nav_msgs::Path>("/mpcc/global_path", 1);
  mpcc_local_path_pub_ = node_handle.advertise<nav_msgs::Path>("/mpcc/local_path", 1);
  // ==================================

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

  s_info_.setZero();
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

    // if(mpcc_thread_enabled_)
    // {
    //   mpcc_global_path_pub_.publish(mpcc_global_path_);
    //   mpcc_local_path_pub_.publish(mpcc_local_path_);
    // }
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
    std::cout << "time     : " << std::fixed << std::setprecision(5) << play_time_.toSec() << std::endl;
		std::cout << "q now    :\t";
		std::cout << std::fixed << std::setprecision(5) << q_.transpose() << std::endl;
		std::cout << "q desired:\t";
		std::cout << std::fixed << std::setprecision(5) << q_desired_.transpose() << std::endl;
		std::cout << "x        :\t";
		std::cout << x_.transpose() << std::endl;
		std::cout << "R        :\t" << std::endl;
		std::cout << std::fixed << std::setprecision(5) << rotation_ << std::endl;
    // std::cout << "J        :\t" << std::endl;
		// std::cout << std::fixed << std::setprecision(5) << j_ << std::endl;
    std::cout << "x_dot    :\t";
		std::cout << x_dot_.norm() << std::endl;
    if(mpcc_thread_enabled_)
    {
      std::cout << "s        :\t";
		  std::cout << s_info_.s << std::endl;
      std::cout << "vs       :\t";
		  std::cout << s_info_.vs << std::endl;
      std::cout << "dVs      :\t";
		  std::cout << s_info_.dVs << std::endl;
    }
    std::cout << "torque desired    :\t";
		std::cout << std::fixed << std::setprecision(5) << torque_desired_.transpose() << std::endl;

    std::cout << "-------------------------------------------------------------------\n\n" << std::endl;
  }
}

Eigen::Matrix<double, 7, 1> jh_controller::PDControl(const Eigen::Matrix<double, 7, 1> & q_desired, const Eigen::Matrix<double, 7, 1> & qdot_desired)
{
  Eigen::Vector7d Kp_diag, Kv_diag;
  Kp_diag << 600.0, 600.0, 600.0, 600.0, 1000.0, 1000.0, 2000.0;
  Kv_diag << 50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0;

  // double kp, kv;
  // kp = 1600;
  // kv = 10;

  // return m_ * ( kp*(q_desired - q_) + kv*(qdot_desired - qdot_)) + c_;
  return m_ * ( Kp_diag.asDiagonal()*(q_desired - q_) + Kv_diag.asDiagonal()*(qdot_desired - qdot_)) + c_;
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

  torque_desired_ = PDControl(q_desired_, qdot_desired_);
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

  input_mutex_.lock();
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

  // ============== MPCC ==============
  if(mpcc_thread_enabled_)
  {
    s_info_.s += s_info_.vs / hz_;
    s_info_.vs += s_info_.dVs / hz_;
  }
  // ==================================

  input_mutex_.unlock();
}

void jh_controller::setDesiredTorque(const Eigen::Matrix<double, 7, 1> & desired_torque)
{
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(desired_torque(i));
  }
}

void jh_controller::asyncCalculationProc()
  {
    // bench_timer_.reset();
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
      qdot_desired_ = qdot_init_;
      q_desired_ = q_init_;
      mpcc_thread_enabled_ = false;

      if(control_mode_ == HOME)
      {
          std::cout << "================ Mode change: HOME position ===============" <<std::endl;
      }
      else if(control_mode_ == MPCC)
      {
          std::cout << "================ Mode change: MPCC ================" <<std::endl;
          s_info_.setZero();
          mpcc_qdot_desired_.setZero();
          mpcc_dVs_desired_ = 0;

          mpcc::Track track = mpcc::Track(json_paths_.track_path);
          mpcc::TrackPos track_xyzr = track.getTrack(x_init_);
          mpc_->setTrack(track_xyzr.X,track_xyzr.Y,track_xyzr.Z,track_xyzr.R);
          mpcc_thread_enabled_ = true;

          mpcc_global_path_.header.frame_id = "panda_link0";
          mpcc_global_path_.poses.clear();
          for(size_t i=0; i<track_xyzr.X.size(); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = track_xyzr.X(i);
            pose.pose.position.y = track_xyzr.Y(i);
            pose.pose.position.z = track_xyzr.Z(i);
            rotationMatrixToQuaternion(track_xyzr.R[i], pose.pose);
            pose.header.frame_id = "panda_link0";

            mpcc_global_path_.poses.push_back(pose);
          }
          mpcc_global_path_pub_.publish(mpcc_global_path_);
      }

    }

    if(control_mode_ == HOME)
    {
      Eigen::Matrix<double, 7, 1> target_q;
      target_q << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
      jh_controller::moveJointPositionTorque(target_q, 5.0);
    }
    else if(control_mode_ == MPCC)
    {
      if(is_mpcc_solved_)
      {
        is_mpcc_solved_ = false;
        qdot_desired_ = mpcc_qdot_desired_;
        s_info_.dVs = mpcc_dVs_desired_;
      }
      q_desired_ += qdot_desired_/hz_;
      torque_desired_ = PDControl(q_desired_, qdot_desired_);
    }
    else
    {
      torque_desired_ = c_;
    }
    calculation_mutex_.unlock();
    // double elapsed_time = bench_timer_.elapsedAndReset();
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
          case 'm':
            jh_controller::setMode(MPCC);
            break;
          default:
            jh_controller::setMode(NONE);
            break;
        }
        calculation_mutex_.unlock();
      }
      
    }
}

void jh_controller::asyncMPCCFProc()
{
  // TODO: use Ts_mpcc_
  std::chrono::milliseconds interval(10);
    while(!quit_all_proc_)
    {
        if(mpcc_thread_enabled_)
        {
          auto start = std::chrono::high_resolution_clock::now();

          mpcc_input_mutex_.lock();
          mpcc::State x0;
          mpcc::StateVector x0_vec;
          // x0_vec << q_, s_info_.s, s_info_.vs;
          x0_vec << q_desired_, s_info_.s, s_info_.vs;
          x0 = mpcc::vectorToState(x0_vec);

          mpcc_input_mutex_.unlock();
          mpcc::MPCReturn mpc_sol = mpc_->runMPC(x0);

          // std::cout << "computation time :" <<std::endl;
          // std::cout << "\t total      :" << mpc_sol.compute_time.total <<std::endl;
          // std::cout << "\t set qp     :" << mpc_sol.compute_time.set_qp <<std::endl;
          // std::cout << "\t solve qp   :" << mpc_sol.compute_time.solve_qp <<std::endl;
          // std::cout << "\t linesearch :" << mpc_sol.compute_time.get_alpha <<std::endl;
          // std::cout << "sol: " << mpcc::inputToVector(mpc_sol.u0).transpose() << std::endl; 

          mpcc_mutex_.lock();
          s_info_.s = x0.s;
          mpcc_qdot_desired_ = mpcc::inputToJointVector(mpc_sol.u0);
          mpcc_dVs_desired_ = mpc_sol.u0.dVs;
          is_mpcc_solved_ = true;            
          mpcc_mutex_.unlock();

          mpcc_local_path_.header.frame_id = "panda_link0";
          mpcc_local_path_.poses.clear();
          for(size_t i=0; i<mpc_sol.mpc_horizon.size(); i++)
          {
            Eigen::Matrix<double, 3, 1> xk;
            Eigen::Matrix<double, 3, 3> rk;
            xk = mpc_->robot_->getEEPosition(mpcc::stateToJointVector(mpc_sol.mpc_horizon[i].xk));
            rk = mpc_->robot_->getEEOrientation(mpcc::stateToJointVector(mpc_sol.mpc_horizon[i].xk));

            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = xk(0);
            pose.pose.position.y = xk(1);
            pose.pose.position.z = xk(2);
            rotationMatrixToQuaternion(rk, pose.pose);
            pose.header.frame_id = "panda_link0";

            mpcc_local_path_.poses.push_back(pose);
          }
          mpcc_local_path_pub_.publish(mpcc_local_path_);

          auto end = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double, std::milli> elapsed = end - start;
          auto sleep_time = interval - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
          if(sleep_time.count() > 0) std::this_thread::sleep_for(sleep_time);
        }
        // TODO: erase 'else' part
        else
        {
          if(mpcc_trigger_()) int temp=0;
        }
    }
}
// ------------------------------------------------------------------------------------------------




} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::jh_controller,
                       controller_interface::ControllerBase)
