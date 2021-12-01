#pragma once

#include <avatar_dual_controllers/servers/action_server_base.h>
#include <avatar_dual_controllers/utils/dyros_math.h>
#include <avatar_msgs/SetTrajectoryFollowerGain.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <random>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>

#define RobotName "panda_right"

using namespace dyros_math;
class ResidualActionServer : public ActionServerBase
{
public:

  void goalCallback() override;
  void preemptCallback() override;
  
  // std::string active_arm_;
  std::map<std::string, bool> active_arms_;
  std::map<std::string, std::vector<std::string> > joint_names_;
  std::map<std::string, int> start_index_map_;
  std::map<std::string, std::pair<Eigen::VectorXd, Eigen::VectorXd> > arm_gain_map_;
  std::map<int, std::pair<std::string, int> > joint_map_;

  Eigen::Vector7d q_target_;
  Eigen::Vector7d q_desired_;
  Eigen::Vector7d qd_desired_;
  Eigen::Vector7d qdd_desired_;

  int num_dof_ = 7;

  ResidualActionServer(std::string name, ros::NodeHandle &nh, 
                          std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);
                          
  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm, const std::string & arm_name);
  bool computeArmForce(ros::Time time, FrankaModelUpdater &arm, const std::string & arm_name);

  bool setRandomTarget(ros::Time time);
  bool setInitTarget(ros::Time time);

  void initMoveit();
  void setMoveitObstables();
  void generateRandTraj();

  void loadNetwork();
  void writeBuffer(FrankaModelUpdater &arm);
  void computeBackwardDynamicsModel();
  void computeTrainedModel();
  void computeExtTorque(FrankaModelUpdater &arm);
  
  void publishResidual();
  void collisionStateCallback(const std_msgs::Bool::ConstPtr& msg);

  bool first_compute_=true;

  double init_time_;
  Eigen::Vector7d init_q_;
  Eigen::Vector7d init_qd_;

  ros::ServiceServer server_ ;

  // Moveit
  inline static const std::string PLANNING_GROUP="panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  moveit::planning_interface::MoveGroupInterface::Plan random_plan_;
  moveit::planning_interface::MoveGroupInterface::Plan random_plan_next_;

  std::vector<double> q_target_plan_;
  std::vector<double> q_target_plan_candidate_;
  std::vector<double> q_init_plan_; 
  std::vector<double> q_dot_plan_;

  Eigen::VectorXd q_limit_u_;
  Eigen::VectorXd q_limit_l_;

  double cur_time_ = 0.0;
  double traj_init_time_ = 0.0;
  double traj_duration_ = 0.0;
  int total_waypoints_;
  int cur_waypoint_ = 0;
  int traj_num_ = 0;

  bool init_traj_started_ = false;
  bool init_traj_prepared_ = false;
  bool next_traj_prepared_ = false;

  int plan_loop_cnt_ = 0;

  // Neural Network
  static const int num_seq = 10;
  static const int num_features = 2;
  static const int num_joint = 7;

  float ring_buffer_[num_seq*num_features*num_joint];
  float ring_buffer_control_input_[num_seq*num_joint];
  int ring_buffer_idx_ = 0;

  float max_theta_ = 3.14;
  float min_theta_ = -3.14;
  float max_theta_dot_ = 0.5;
  float min_theta_dot_ = -0.5;
  Eigen::Matrix<float, num_joint, 1> output_scaling;

  static const int num_hidden_neurons_ = 100;

  Eigen::Matrix<float, num_hidden_neurons_, num_seq*num_features*num_joint> backward_W0;
  Eigen::Matrix<float, num_hidden_neurons_, 1> backward_b0;
  Eigen::Matrix<float, num_hidden_neurons_, num_hidden_neurons_> backward_W2;
  Eigen::Matrix<float, num_hidden_neurons_, 1> backward_b2;
  Eigen::Matrix<float, num_joint, num_hidden_neurons_> backward_W4;
  Eigen::Matrix<float, num_joint, 1> backward_b4;

  Eigen::Matrix<float, (num_seq-1)*num_features*num_joint, 1> condition_;
  Eigen::Matrix<float, num_features*num_joint, 1> state_;
  Eigen::Matrix<float, num_joint, 1> input_;

  Eigen::Matrix<float, num_hidden_neurons_, 1> backward_layer1_;
  Eigen::Matrix<float, num_hidden_neurons_, 1> backward_layer2_;
  Eigen::Matrix<float, num_joint, 1> backward_network_output_;

  Eigen::Vector7d estimated_ext_torque_NN_;

  // Residual publish
  ros::Publisher resi_publisher_;
  std_msgs::Float32MultiArray resi_msg_;
  float resi_buffer_[num_seq*num_joint];
  int resi_buffer_idx_ = 0;

  // Subscribe collision state
  ros::Subscriber collision_subscriber_;
  bool collision_state_;

  // Force control
  Eigen::Isometry3d x_target_;
  Eigen::Isometry3d x_desired_;
  Eigen::Isometry3d x_mode_init_;
  Eigen::Vector6d x_dot_desired_;

  Eigen::Vector6d estimated_ext_force_;
  Eigen::Vector6d estimated_ext_force_init_;
  
  bool force_control_init_;

  double mode_init_time_ = 0.0;

  double f_I_ = 0.0;
  double f_d_z_ = 0.0;

  Eigen::MatrixXd kv_task_, kp_task_;

private:
  bool setGain(avatar_msgs::SetTrajectoryFollowerGain::Request  &req,
                 avatar_msgs::SetTrajectoryFollowerGain::Response &res);
};
