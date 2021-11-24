#include <avatar_dual_controllers/servers/residual_action_server.h>

ResidualActionServer::ResidualActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu)
{
  std::string joint_name_prefix = "_joint";
  for (auto iter = mu_.begin(); iter != mu_.end(); iter++)
  {
    std::vector<std::string> joint_names;
    for(int i=1; i<=7; i++)
    {
      std::string joint_name = iter->first + joint_name_prefix + std::to_string(i);
      std::cout << joint_name;
      joint_names.push_back(joint_name);
    }
    joint_names_[iter->first] = joint_names;
        
    Eigen::VectorXd kp(7), kv(7);
    kp << 800, 800, 800, 800, 500, 400, 300;
    kv << 10, 10, 10, 10, 5, 5, 3;
    // kp << 400, 400, 400, 400, 300, 200, 100;
    // kv << 5, 5, 5, 5, 2, 2, 1;
    active_arms_[iter->first] = true;
    
    arm_gain_map_[iter->first] = std::make_pair(kp,kv);

    if (iter->first == RobotName)
      control_running_ = true;
  }

  server_  = nh_.advertiseService(name + "_gain_set", &ResidualActionServer::setGain, this);
}

void ResidualActionServer::goalCallback()
{

}
void ResidualActionServer::preemptCallback()
{
  
}


bool ResidualActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;


  for (auto & arm : active_arms_)
  {
    if (arm.first == RobotName)
    {
      if (first_compute_)
      {
        init_time_ = time.toSec();
        init_q_ = mu_[arm.first]->q_;
        init_qd_ = mu_[arm.first]->qd_;
        first_compute_ = false;
      }
      setTarget(time);    
      
      if (arm.second == true)
      {
        computeArm(time, *mu_[arm.first], arm.first);
      }
    }
  }
  return false;
}


bool ResidualActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm, const std::string & arm_name)
{
  Eigen::Matrix<double,7,1> desired_torque;
  
  Eigen::Matrix<double, 7,7> kp, kv;
  
  kp = arm_gain_map_[arm_name].first.asDiagonal();
  kv = arm_gain_map_[arm_name].second.asDiagonal();

  desired_torque = (kp*(q_desired_ - arm.q_) + kv*(qd_desired_ - arm.qd_)) + arm.coriolis_;

  if (++ print_count_ > iter_per_print_)
  {
    Eigen::IOFormat tab_format(Eigen::FullPrecision, 0, "\t", "\n");
    debug_file_.precision(std::numeric_limits< double >::digits10);
    if (debug_file_.is_open())
    {
      debug_file_ << arm.arm_name_ << '\t' 
            << time.toSec() << '\t' 
            << arm.q_.transpose().format(tab_format) << '\t'
            << arm.qd_.transpose().format(tab_format) << '\t' 
            << arm.tau_ext_filtered_.transpose().format(tab_format) << '\t' 
            << arm.mob_torque_.transpose().format(tab_format) << '\t' 
            << q_desired_.transpose().format(tab_format) << '\t' 
            << qd_desired_.transpose().format(tab_format)
            << std::endl;
    }
    print_count_ = 0;
  }

  arm.setTorque(desired_torque);
  return true;
}

bool ResidualActionServer::setTarget(ros::Time time)
{
  q_target_ << 0, 0, 0, -90*DEG2RAD, 0, 90*DEG2RAD, 0;

  for (int i = 0; i < 7; i++)
  {
    q_desired_(i) = dyros_math::cubic(time.toSec(), init_time_, init_time_ + 5.0, init_q_(i), q_target_(i), 0.0, 0.0);
    qd_desired_(i) = dyros_math::cubic(time.toSec(), init_time_, init_time_ + 5.0, init_qd_(i), q_target_(i), 0.0, 0.0);
  }     

  return true;
}


bool ResidualActionServer::setGain(avatar_msgs::SetTrajectoryFollowerGain::Request  &req,
                                            avatar_msgs::SetTrajectoryFollowerGain::Response &res)
{
    auto it = arm_gain_map_.find(req.arm_name);
  if (it == arm_gain_map_.end())
  {
    ROS_WARN("arm name %s is not in the arm_gain_map_. ignore", req.arm_name.c_str());
    res.is_succeed = false;
    return true;
  }
  
  if (req.p_gain.size() != 7 || req.d_gain.size() != 7)
  {
    ROS_INFO("req.p_gain != 7, resetting the gains");

    Eigen::VectorXd kp(7), kv(7);
    kp << 800, 800, 800, 800, 500, 400, 300;
    kv << 10, 10, 10, 10, 5, 5, 3;
    arm_gain_map_[req.arm_name] = std::make_pair(kp,kv);
  }
  else
  {
    arm_gain_map_[req.arm_name].first = Eigen::VectorXd::Map(req.p_gain.data(),7);
    arm_gain_map_[req.arm_name].second = Eigen::VectorXd::Map(req.d_gain.data(),7);
  }
  res.is_succeed = true;
  return true;
}
