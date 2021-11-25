#include <avatar_dual_controllers/servers/residual_action_server.h>

ResidualActionServer::ResidualActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu), move_group_(PLANNING_GROUP)
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

    // Moveit
    ros::AsyncSpinner spinner(1);
    spinner.start();
    initMoveit();
  }

  server_  = nh_.advertiseService(name + "_gain_set", &ResidualActionServer::setGain, this);
}

void ResidualActionServer::goalCallback()
{

}
void ResidualActionServer::preemptCallback()
{
  
}

void ResidualActionServer::initMoveit()
{
    // For Moveit
    q_limit_l_.resize(num_dof_);
    q_limit_u_.resize(num_dof_);
    q_limit_l_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0873, -2.8973;
    q_limit_u_ <<  2.8973,  1.7628,  2.8973,  0.0698,  2.8973,  2.1127,  2.8973;

    q_target_plan_.resize(num_dof_);
    q_target_plan_candidate_.resize(num_dof_);
    q_init_plan_.resize(num_dof_);
    q_dot_plan_.resize(num_dof_);
    // std::fill(q_target_plan_.begin(), q_target_plan_.end(), 0);
    q_target_plan_[0] = 0.0;
    q_target_plan_[1] = 0.0;
    q_target_plan_[2] = 0.0;
    q_target_plan_[3] = -90*DEG2RAD;
    q_target_plan_[4] = 0.0;
    q_target_plan_[5] = 90*DEG2RAD;
    q_target_plan_[6] = 0.0;
    std::fill(q_init_plan_.begin(), q_init_plan_.end(), 0);
    std::fill(q_dot_plan_.begin(), q_dot_plan_.end(), 0);

    const robot_state::JointModelGroup* joint_model_group = move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    move_group_.setMaxVelocityScalingFactor(1.0);


    setMoveitObstables();
}

void ResidualActionServer::setMoveitObstables()
{
    // Create collision object
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);

    double square_len = 4.0;
    double hole_len = 0.22;
    double y_offset = 0.0;
    double height_offset = 0.0;
    // Box 1
    primitive.dimensions[0] = (square_len - hole_len)/2.0;
    primitive.dimensions[1] = square_len;
    primitive.dimensions[2] = 0.01;
    moveit_msgs::CollisionObject box1;
    box1.header.frame_id = move_group_.getPlanningFrame();
    box1.id = "box1";
    geometry_msgs::Pose box1_pose;
    box1_pose.position.x = (hole_len + primitive.dimensions[0])/2.0;
    box1_pose.position.y = y_offset;
    box1_pose.position.z = height_offset + 0.095;
    box1.primitives.push_back(primitive);
    box1.primitive_poses.push_back(box1_pose);
    box1.operation = box1.ADD;
    collision_objects.push_back(box1);

    // Box 2
    moveit_msgs::CollisionObject box2;
    box2.header.frame_id = move_group_.getPlanningFrame();
    box2.id = "box2";
    geometry_msgs::Pose box2_pose;
    box2_pose.position.x = -(hole_len + primitive.dimensions[0])/2.0;
    box2_pose.position.y = y_offset;
    box2_pose.position.z = height_offset + 0.095;
    box2.primitives.push_back(primitive);
    box2.primitive_poses.push_back(box2_pose);
    box2.operation = box2.ADD;
    collision_objects.push_back(box2);

    // Box 3
    primitive.dimensions[0] = hole_len;
    primitive.dimensions[1] = (square_len - hole_len)/2.0;;
    moveit_msgs::CollisionObject box3;
    box3.header.frame_id = move_group_.getPlanningFrame();
    box3.id = "box3";
    geometry_msgs::Pose box3_pose;
    box3_pose.position.x = 0.0;
    box3_pose.position.y = y_offset + (primitive.dimensions[1] + hole_len) / 2.0;
    box3_pose.position.z = height_offset + 0.095;
    box3.primitives.push_back(primitive);
    box3.primitive_poses.push_back(box3_pose);
    box3.operation = box3.ADD;
    collision_objects.push_back(box3);

    // Box 4
    moveit_msgs::CollisionObject box4;
    box4.header.frame_id = move_group_.getPlanningFrame();
    box4.id = "box4";
    geometry_msgs::Pose box4_pose;
    box4_pose.position.x = 0.0;
    box4_pose.position.y = y_offset + -(primitive.dimensions[1] + hole_len) / 2.0;;
    box4_pose.position.z = height_offset + 0.095;
    box4.primitives.push_back(primitive);
    box4.primitive_poses.push_back(box4_pose);
    box4.operation = box4.ADD;
    collision_objects.push_back(box4);

    planning_scene_interface_.addCollisionObjects(collision_objects);
}

void ResidualActionServer::generateRandTraj()
{
    if (!next_traj_prepared_)
    {
        moveit::core::RobotState start_state = *(move_group_.getCurrentState());
        for (int i = 0; i < num_dof_; i++)
        {
            q_init_plan_[i] = q_target_plan_[i];
            q_dot_plan_[i] = 0.0;
        }

        start_state.setJointGroupPositions(PLANNING_GROUP, q_init_plan_);
        start_state.setJointGroupVelocities(PLANNING_GROUP, q_dot_plan_);
        move_group_.setStartState(start_state);

        std::random_device rand_device;
        std::default_random_engine rand_seed;
        std::uniform_real_distribution<double> angles[num_dof_];
        rand_seed.seed(rand_device());
        
        double safe_range = 0.0;
        for (size_t i = 0; i < num_dof_; i++)
        {
            safe_range = ((q_limit_u_[i] - q_limit_l_[i]) * 0.1);
            angles[i] = std::uniform_real_distribution<double>((q_limit_l_[i] + safe_range), (q_limit_u_[i] - safe_range));
        }
        
        for (int i = 0; i < num_dof_; i++) 
            q_target_plan_candidate_[i] = ((angles[i])(rand_seed));
  
        move_group_.setJointValueTarget(q_target_plan_candidate_);

        if (move_group_.plan(random_plan_next_) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
          q_target_plan_ = q_target_plan_candidate_;
          next_traj_prepared_ = true;

          if (traj_num_ == 0)
            init_traj_prepared_ = true;

          traj_num_++;
          std::cout<< "Trajectory " << traj_num_ << " prepared"<<std::endl;
        }
    }
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
      generateRandTraj();
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
  // desired_torque.setZero();

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
