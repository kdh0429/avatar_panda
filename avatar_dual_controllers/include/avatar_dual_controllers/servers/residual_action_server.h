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

  int num_dof_ = 7;

  ResidualActionServer(std::string name, ros::NodeHandle &nh, 
                          std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);
                          
  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm, const std::string & arm_name);

  bool setTarget(ros::Time time);

  void initMoveit();
  void setMoveitObstables();
  void generateRandTraj();

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

  double traj_init_time_ = 0.0;
  double traj_duration_ = 0.0;
  int total_waypoints_;
  int cur_waypoint_ = 0;
  int traj_num_ = 0;

  bool init_traj_prepared_ = false;
  bool next_traj_prepared_ = false;

private:
  bool setGain(avatar_msgs::SetTrajectoryFollowerGain::Request  &req,
                 avatar_msgs::SetTrajectoryFollowerGain::Response &res);
};
