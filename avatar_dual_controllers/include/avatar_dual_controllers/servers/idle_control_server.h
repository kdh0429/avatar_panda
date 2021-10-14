#pragma once

#include <ros/ros.h>

#include <avatar_dual_controllers/utils/dyros_math.h>
#include <avatar_dual_controllers/utils/model/franka_model_updater.h>
#include <avatar_msgs/IdleControl.h>

#include <avatar_dual_controllers/utils/control/peg_in_hole_base.h>

#include <Eigen/Dense>
#include <map>

using namespace dyros_math;

class IdleControlServer
{
  ros::NodeHandle & nh_; 
  std::map<std::string, std::shared_ptr<FrankaModelUpdater> > mu_;
  std::map<std::string, avatar_msgs::IdleControl::Request > params_;
  
  ros::ServiceServer server_ ;

public:
  IdleControlServer(const std::string &name, ros::NodeHandle &nh,
                  std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time);
  void computeArm(ros::Time time, FrankaModelUpdater &arm, avatar_msgs::IdleControl::Request &param);

private:
  bool setTarget(avatar_msgs::IdleControl::Request  &req,
                 avatar_msgs::IdleControl::Response &res);
};