#pragma once

#include <avatar_dual_controllers/servers/action_server_base.h>
#include <avatar_dual_controllers/utils/dyros_math.h>
#include <avatar_msgs/SetTrajectoryFollowerGain.h>

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

  ResidualActionServer(std::string name, ros::NodeHandle &nh, 
                          std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);
                          
  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm, const std::string & arm_name);

  bool setTarget(ros::Time time);

  bool first_compute_=true;

  double init_time_;
  Eigen::Vector7d init_q_;
  Eigen::Vector7d init_qd_;

  ros::ServiceServer server_ ;

private:
  bool setGain(avatar_msgs::SetTrajectoryFollowerGain::Request  &req,
                 avatar_msgs::SetTrajectoryFollowerGain::Response &res);
};
