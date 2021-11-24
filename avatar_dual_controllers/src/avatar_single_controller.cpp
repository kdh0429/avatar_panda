#include <avatar_dual_controllers/avatar_single_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace avatar_dual_controllers {

bool AvatarSingleController::initArm(
    hardware_interface::RobotHW* robot_hw,
    const std::string& arm_id,
    const std::vector<std::string>& joint_names) {
  std::shared_ptr<FrankaModelUpdater> arm_data = std::make_shared<FrankaModelUpdater>();
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "AvatarSingleController: Error getting model interface from hardware");
    return false;
  }
  try {
    arm_data->model_handle_ = std::make_shared<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "AvatarSingleController: Exception getting model handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "AvatarSingleController: Error getting state interface from hardware");
    return false;
  }
  try {
    arm_data->state_handle_ = std::make_shared<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "AvatarSingleController: Exception getting state handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "AvatarSingleController: Error getting effort joint interface from "
        "hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      arm_data->joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "AvatarSingleController: Exception getting joint handles: "
          << ex.what());
      return false;
    }
  }
  arm_data->arm_name_ = arm_id;
  arms_data_.emplace(std::make_pair(arm_id, arm_data));
  return true;
}

bool AvatarSingleController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {

  if (!node_handle.getParam("right/arm_id", right_arm_id_)) {
    ROS_ERROR_STREAM(
        "AvatarSingleController: Could not read parameter right_arm_id_");
    return false;
  }

  std::vector<std::string> right_joint_names;
  if (!node_handle.getParam("right/joint_names", right_joint_names) ||
      right_joint_names.size() != 7) {
    ROS_ERROR(
        "AvatarSingleController: Invalid or no right_joint_names parameters "
        "provided, "
        "aborting controller init!");
    return false;
  }

  bool right_success = initArm(robot_hw, right_arm_id_, right_joint_names);

  residual_action_server_ = std::make_unique<ResidualActionServer>
  ("/avatar_dual_controller/residual_control", node_handle, arms_data_);

  // idle_control_server_ = std::make_unique<IdleControlServer>
  // ("/avatar_dual_controller/idle_control", node_handle, arms_data_);

  return right_success;
}


void AvatarSingleController::startingArm(FrankaModelUpdater& arm_data) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = arm_data.state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Isometry3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  sb_.reset();
}

void AvatarSingleController::starting(const ros::Time& time) {
  for (auto& arm_data : arms_data_) {
    startingArm(*arm_data.second);
  }
  
  start_time_ = ros::Time::now();
}

void AvatarSingleController::update(const ros::Time& time, const ros::Duration& period) {

  // std::cout << period.toSec() << std::endl;
  double t[30];
  t[0] = sb_.elapsedAndReset();
  for (auto& arm : arms_data_) {
    arm.second->updateModel();
    arm.second->target_updated_ = false;
  }
  t[1] = sb_.elapsedAndReset();

  int ctr_index = 2;
  residual_action_server_->compute(time);
  t[ctr_index++] = sb_.elapsedAndReset();

  // add normal action server above ----------
  // idle_control_server_->compute(time);
  // t[ctr_index++] = sb_.elapsedAndReset();


  for(int i=0; i<ctr_index; ++i)
  {
    debug_file_td_ << t[i] << '\t';
  }
  debug_file_td_ << std::endl;
  
}

Eigen::Matrix<double, 7, 1> AvatarSingleController::saturateTorqueRate(
    const FrankaModelUpdater& arm_data,
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, arm_data.delta_tau_max_),
                                               -arm_data.delta_tau_max_);
  }
  return tau_d_saturated;
}


}  // namespace avatar_dual_controllers

PLUGINLIB_EXPORT_CLASS(avatar_dual_controllers::AvatarSingleController,
                       controller_interface::ControllerBase)
