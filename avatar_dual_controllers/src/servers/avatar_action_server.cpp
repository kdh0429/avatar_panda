#include <avatar_dual_controllers/servers/avatar_action_server.h>

AvatarActionServer::AvatarActionServer(std::string name, ros::NodeHandle &nh, 
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
    if (iter->first == SLAVE)
    {
      kp << 800, 800, 800, 800, 500, 400, 300;
      kv << 10, 10, 10, 10, 5, 5, 3;
      // kp << 400, 400, 400, 400, 300, 200, 100;
      // kv << 5, 5, 5, 5, 2, 2, 1;
      active_arms_[iter->first] = true;
      slave_on_ = true;
    }
    else
    {
      kp << 0, 0, 0, 0, 0, 0, 0;
      kv << 0, 0, 0, 0, 0, 0, 0;
      active_arms_[iter->first] = true;
      master_on_ = true;
    }
    arm_gain_map_[iter->first] = std::make_pair(kp,kv);
  }

  tau_feedback_desired_master_.setZero();

  if (slave_on_ && master_on_)
    control_running_ = true;

  server_  = nh_.advertiseService(name + "_slave_gain_set", &AvatarActionServer::setSlaveGain, this);
}

void AvatarActionServer::goalCallback()
{

}
void AvatarActionServer::preemptCallback()
{
  
}


bool AvatarActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;


  for (auto & arm : active_arms_)
  {
    if (arm.first == MASTER)
    {
      if (master_first_compute_)
      {
        init_time_ = time.toSec();
        init_master_q_ = mu_[arm.first]->q_;
        init_master_qd_ = mu_[arm.first]->qd_;
        master_first_compute_ = false;
      }
      setSlaveTarget(time, *mu_[arm.first]);
    }
    else if (arm.first == SLAVE)
    {
      if (slave_first_compute_)
      {
        init_slave_q_ = mu_[arm.first]->q_;
        init_slave_qd_ = mu_[arm.first]->qd_;
        slave_first_compute_ = false;
      }
      setMasterTarget(time, *mu_[arm.first]);
    }

    if (arm.second == true)
    {
      computeArm(time, *mu_[arm.first], arm.first);
    }
  }
  return false;
}


bool AvatarActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm, const std::string & arm_name)
{
  Eigen::Matrix<double,7,1> desired_torque;
  
  if (arm_name == SLAVE)
  {
    Eigen::Matrix<double, 7,7> kp, kv;
    
    kp = arm_gain_map_[arm_name].first.asDiagonal();
    kv = arm_gain_map_[arm_name].second.asDiagonal();

    desired_torque = (kp*(q_desired_slave_ - arm.q_) + kv*(qd_desired_slave_ - arm.qd_)) + arm.coriolis_;
  }
  else if (arm_name == MASTER)
  {
    desired_torque = -tau_feedback_desired_master_;
    // desired_torque.setZero();
  }
  
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
            << q_desired_slave_.transpose().format(tab_format) << '\t' 
            << qd_desired_slave_.transpose().format(tab_format)
            << std::endl;
    }
    print_count_ = 0;
  }

  arm.setTorque(desired_torque);
  return true;
}

bool AvatarActionServer::setSlaveTarget(ros::Time time, FrankaModelUpdater &master_arm)
{
  if (time.toSec() < init_time_ + 5.0)
  {
    for (int i = 0; i < 7; i++)
    {
      q_desired_slave_(i) = dyros_math::cubic(time.toSec(), init_time_, init_time_ + 5.0, init_slave_q_(i), master_arm.q_(i), 0.0, 0.0);
      qd_desired_slave_(i) = dyros_math::cubic(time.toSec(), init_time_, init_time_ + 5.0, init_slave_qd_(i), master_arm.qd_(i), 0.0, 0.0);
    }      
  }
  else
  {
    q_desired_slave_ = master_arm.q_;
    qd_desired_slave_ = master_arm.qd_;
  }

  // std::cout << "q des slave: " << q_desired_slave_.transpose() << std::endl;
  // std::cout << "q init master: " << init_master_q_.transpose() << std::endl;
  // std::cout << "q init slave: " << init_slave_q_.transpose() << std::endl;

  return true;
}

bool AvatarActionServer::setMasterTarget(ros::Time time, FrankaModelUpdater &slave_arm)
{
  tau_feedback_desired_master_ = slave_arm.tau_ext_filtered_;

  return true;
}



bool AvatarActionServer::setSlaveGain(avatar_msgs::SetTrajectoryFollowerGain::Request  &req,
                                            avatar_msgs::SetTrajectoryFollowerGain::Response &res)
{
  if (req.arm_name == MASTER)
  {
    ROS_WARN("Arm name %s is the master. Only setting slave gain is available.", req.arm_name.c_str());
    res.is_succeed = false;
    return true;
  }
  else if (req.arm_name == SLAVE)
  {
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
}
