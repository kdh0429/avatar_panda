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
    // kp << 800, 800, 800, 800, 500, 400, 300;
    // kv << 10, 10, 10, 10, 5, 5, 3;
    kp << 400, 400, 400, 400, 300, 200, 100;
    kv << 20, 20, 20, 20, 10, 10, 3;
    active_arms_[iter->first] = true;
    
    arm_gain_map_[iter->first] = std::make_pair(kp,kv);

    if (iter->first == RobotName)
      control_running_ = true;

    // Moveit
    ros::AsyncSpinner spinner(1);
    spinner.start();
    initMoveit();

    // Neural Network
    loadNetwork();
    std::fill(ring_buffer_, ring_buffer_+buffer_idx_size*num_features*num_joint, 0);
    std::fill(ring_buffer_control_input_, ring_buffer_control_input_+buffer_idx_size*num_joint, 0);
    std::fill(resi_buffer_, resi_buffer_+buffer_idx_size*num_joint, 0);
    output_scaling << 10.246, 54.954, 27.211, 22.156, 2.6118, 2.6689, 0.47224;

    // Residual Publish
    resi_publisher_ = nh.advertise<std_msgs::Float32MultiArray>("/panda/residual", 1000);
    resi_msg_.data.resize(num_joint*num_seq);

    // Subscribe Collision State
    collision_subscriber_ = nh.subscribe("/panda/collision_state", 1000,  &ResidualActionServer::collisionStateCallback, this);

    // Force Control
    kp_task_.resize(6,6);
    kp_task_.setZero();
    kv_task_.resize(6,6);
    kv_task_.setZero();

    for (int i = 0; i < 3; i++)
    {
        kp_task_(i,i) = 1000;
        kv_task_(i,i) = 20;
        kp_task_(i+3,i+3) = 4000;
        kv_task_(i+3,i+3) = 30;
    }
  }

  server_  = nh_.advertiseService(name + "_gain_set", &ResidualActionServer::setGain, this);

  openDebugFile("/home/dyros21/avatar_panda/random_free");
}

void ResidualActionServer::goalCallback()
{

}
void ResidualActionServer::preemptCallback()
{
  
}

void ResidualActionServer::initMoveit()
{
    move_group_.setPlanningTime(0.015);
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

    move_group_.setMaxVelocityScalingFactor(0.4);


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
    if (current_traj_finished_ && plan_loop_cnt_%100 == 0)
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
          current_traj_finished_ = false;
          plan_loop_cnt_ = 0;

          if (traj_num_ == 0)
          {
            init_traj_prepared_ = true;
          }

          traj_num_++;
          std::cout<< "Trajectory " << traj_num_ << " prepared"<<std::endl;
        }
    }
    plan_loop_cnt_++;
}

// Neural Network
void ResidualActionServer::loadNetwork()
{
    std::ifstream file[12];
    file[0].open("/home/dyros21/avatar_panda/src/avatar_dual_controllers/NNweight/backward_network_0_weight.txt", std::ios::in);
    file[1].open("/home/dyros21/avatar_panda/src/avatar_dual_controllers/NNweight/backward_network_0_bias.txt", std::ios::in);
    file[2].open("/home/dyros21/avatar_panda/src/avatar_dual_controllers/NNweight/backward_network_2_weight.txt", std::ios::in);
    file[3].open("/home/dyros21/avatar_panda/src/avatar_dual_controllers/NNweight/backward_network_2_bias.txt", std::ios::in);
    file[4].open("/home/dyros21/avatar_panda/src/avatar_dual_controllers/NNweight/backward_network_4_weight.txt", std::ios::in);
    file[5].open("/home/dyros21/avatar_panda/src/avatar_dual_controllers/NNweight/backward_network_4_bias.txt", std::ios::in);

    if(!file[0].is_open())
    {
        std::cout<<"Can not find the weight file"<<std::endl;
    }

    float temp;
    int row = 0;
    int col = 0;

    while(!file[0].eof() && row != backward_W0.rows())
    {
        file[0] >> temp;
        if(temp != '\n')
        {
            backward_W0(row, col) = temp;
            col ++;
            if (col == backward_W0.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[1].eof() && row != backward_b0.rows())
    {
        file[1] >> temp;
        if(temp != '\n')
        {
            backward_b0(row, col) = temp;
            col ++;
            if (col == backward_b0.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[2].eof() && row != backward_W2.rows())
    {
        file[2] >> temp;
        if(temp != '\n')
        {
            backward_W2(row, col) = temp;
            col ++;
            if (col == backward_W2.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[3].eof() && row != backward_b2.rows())
    {
        file[3] >> temp;
        if(temp != '\n')
        {
            backward_b2(row, col) = temp;
            col ++;
            if (col == backward_b2.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[4].eof() && row != backward_W4.rows())
    {
        file[4] >> temp;
        if(temp != '\n')
        {
            backward_W4(row, col) = temp;
            col ++;
            if (col == backward_W4.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[5].eof() && row != backward_b4.rows())
    {
        file[5] >> temp;
        if(temp != '\n')
        {
            backward_b4(row, col) = temp;
            col ++;
            if (col == backward_b4.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
}

void ResidualActionServer::writeBuffer(FrankaModelUpdater &arm)
{
  for (int i = 0; i < num_dof_; i++)
  {
      ring_buffer_[ring_buffer_idx_*num_features*num_joint + num_features*i] = 2*(arm.q_(i)-min_theta_)/(max_theta_-min_theta_) - 1;
      ring_buffer_[ring_buffer_idx_*num_features*num_joint + num_features*i + 1] = 2*(arm.qd_(i)-min_theta_dot_)/(max_theta_dot_-min_theta_dot_) - 1;
      ring_buffer_control_input_[ring_buffer_idx_*num_joint + i] = 2*(arm.tau_measured_(i)+output_scaling(i))/(output_scaling(i)+output_scaling(i)) - 1;
  }
  
  ring_buffer_idx_++;
  if (ring_buffer_idx_ == buffer_idx_size)
      ring_buffer_idx_ = 0;

  for (int i = 0; i < num_dof_; i++)
  {
      resi_buffer_[resi_buffer_idx_*num_joint + i] = estimated_ext_torque_NN_(i);
  }
  
  resi_buffer_idx_++;
  if (resi_buffer_idx_ == buffer_idx_size)
      resi_buffer_idx_ = 0;
}

void ResidualActionServer::computeBackwardDynamicsModel()
{
    Eigen::Matrix<float, num_seq*num_features*num_joint, 1> backNetInput;
    
    backNetInput << condition_, state_;
    
    backward_layer1_ = backward_W0 * backNetInput + backward_b0;
    for (int i = 0; i < num_hidden_neurons_; i++) 
    {
        if (backward_layer1_(i) < 0)
            backward_layer1_(i) = 0.0;
    }

    backward_layer2_ = backward_W2 * backward_layer1_ + backward_b2;
    for (int i = 0; i < num_hidden_neurons_; i++) 
    {
        if (backward_layer2_(i) < 0)
            backward_layer2_(i) = 0.0;
    }

    backward_network_output_ = backward_W4 * backward_layer2_ + backward_b4;
} 

// Residual Publish
void ResidualActionServer::publishResidual()
{
    int cur_idx = 0;
    if (resi_buffer_idx_ == 0)
        cur_idx = buffer_idx_size - 1;
    else
        cur_idx = resi_buffer_idx_ - 1;

    for (int seq = 0; seq < num_seq; seq++)
    {
        for (int i=0; i < num_joint; i++)
        {
            int process_data_idx = cur_idx + train_test_time_ratio*seq + 1;
            if (process_data_idx >= buffer_idx_size)
                process_data_idx -= buffer_idx_size;
           resi_msg_.data[num_joint*seq + i] = resi_buffer_[process_data_idx*num_joint + i];
        }
    }
    resi_publisher_.publish(resi_msg_);
}

void ResidualActionServer::collisionStateCallback(const std_msgs::Bool::ConstPtr& msg)
{
  collision_state_ = msg->data;
  if (collision_state_ == true)
    std::cout<<"Collision Ocurred!" << std::endl;
}

void ResidualActionServer::computeTrainedModel()
{
  int cur_idx = 0;
  if (ring_buffer_idx_ == 0)
      cur_idx = buffer_idx_size - 1;
  else
      cur_idx = ring_buffer_idx_ - 1;

  for (int seq = 0; seq < num_seq-1; seq++)
  {
      for (int input_feat = 0; input_feat < num_features*num_joint; input_feat++)
      {
          int process_data_idx = cur_idx + train_test_time_ratio*seq + 1;
          if (process_data_idx >= buffer_idx_size)
              process_data_idx -= buffer_idx_size;
          condition_(seq*num_features*num_joint + input_feat) = ring_buffer_[process_data_idx*num_features*num_joint + input_feat];
      }
  }
  for (int input_feat = 0; input_feat < num_features*num_joint; input_feat++)
  {
      state_(input_feat) = ring_buffer_[cur_idx*num_features*num_joint + input_feat];
  }
  for (int i = 0; i < num_joint; i++)
      input_(i) = ring_buffer_control_input_[cur_idx*num_joint + i];

  computeBackwardDynamicsModel();
}

void ResidualActionServer::computeExtTorque(FrankaModelUpdater &arm)
{
    int cur_idx = 0;
    if (ring_buffer_idx_ == 0)
        cur_idx = buffer_idx_size - 1;
    else
        cur_idx = ring_buffer_idx_ - 1;

    for (int i= 0; i < num_dof_; i++)
    {
        estimated_ext_torque_NN_(i) = ring_buffer_control_input_[cur_idx*num_joint + i] - backward_network_output_(i);
        estimated_ext_torque_NN_(i) = estimated_ext_torque_NN_(i) * output_scaling(i);
    }

    estimated_ext_force_ = arm.jacobian_bar_.transpose() * estimated_ext_torque_NN_;
}

bool ResidualActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;


  for (auto & arm : active_arms_)
  {
    if (arm.first == RobotName)
    {
      cur_time_ = time.toSec();

      if (first_compute_)
      {
        init_time_ = time.toSec();
        init_q_ = mu_[arm.first]->q_;
        init_qd_ = mu_[arm.first]->qd_;

        first_compute_ = false;
      }

      // Random Trajectory
      if (control_mode_ == RandomMotion)
      { 
        if (cur_time_ < init_time_ + 5.0)
        {
          setInitTarget(time);
          for (int i=0; i< num_dof_; i++)
            q_target_plan_[i] =  mu_[arm.first]->q_(i);
        }
        else
        {
          generateRandTraj();
        }
        
        if (init_traj_prepared_)
        {
          setRandomTarget(time);  
        }   
        else
        {
          setInitTarget(time);
        }
        
        if (arm.second == true)
        {
          computeArm(time, *mu_[arm.first], arm.first);
          writeBuffer(*mu_[arm.first]);
          computeTrainedModel();
          computeExtTorque(*mu_[arm.first]);
          publishResidual();
        }
      }

      // Force Control
      if (control_mode_ == HybridControl)
      {
        if (cur_time_ < init_time_ + 5.0)
        {
          setInitTarget(time);
          computeArm(time, *mu_[arm.first], arm.first);
        }
        else
        {
          if (arm.second == true)
          {
            computeArmForce(time, *mu_[arm.first], arm.first);       
          }
        }
        writeBuffer(*mu_[arm.first]);
        computeTrainedModel();
        computeExtTorque(*mu_[arm.first]);
        publishResidual();    
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

  desired_torque = arm.modified_mass_matrix_*((kp*(q_desired_ - arm.q_) + kv*(qd_desired_ - arm.qd_))) + arm.coriolis_;

  if (collision_state_)
    desired_torque.setZero();

  // Eigen::Matrix<double, 7,7> kpp = Eigen::Matrix<double, 7,7>::Identity() * 0.2;
  // kpp(6,6) = 0.05;
  // desired_torque = kpp * qdd_desired_ + (kp*(q_desired_ - arm.q_) + kv*(qd_desired_ - arm.qd_)) + arm.coriolis_;

  if (++ print_count_ > iter_per_print_)
  {
    debug_file_ << std::fixed << std::setprecision(8);
    Eigen::IOFormat tab_format(Eigen::StreamPrecision, 0, "\t", "\n");
    // debug_file_.precision(std::numeric_limits< double >::digits10);
    if (debug_file_.is_open())
    {
      // debug_file_ << time.toSec() - init_time_ << '\t' 
      //       << arm.q_.transpose().format(tab_format) << '\t'
      //       << arm.qd_.transpose().format(tab_format) << '\t' 
      //       << arm.tau_measured_.transpose().format(tab_format) << '\t' 
      //       << arm.tau_desired_read_.transpose().format(tab_format) << '\t' 
      //       << arm.tau_ext_filtered_.transpose().format(tab_format) << '\t' 
      //       << q_desired_.transpose().format(tab_format) << '\t' 
      //       << qd_desired_.transpose().format(tab_format)
      //       << std::endl;
    }

    // std::cout<<"q: " << arm.q_.transpose() << std::endl;
    // std::cout<<"qdot: " << arm.qd_.transpose() << std::endl;
    // std::cout<<"tau: " << arm.tau_measured_.transpose() << std::endl;
    // std::cout<<"Nework Output: " << backward_network_output_.transpose() << std::endl;
    // std::cout<<"Ext Torque: " << estimated_ext_torque_NN_.transpose() << std::endl;
    // std::cout<<"Ext Force: " << estimated_ext_force_.transpose() << std::endl;
    print_count_ = 0;
  }

  arm.setTorque(desired_torque);
  return true;
}

bool ResidualActionServer::computeArmForce(ros::Time time, FrankaModelUpdater &arm, const std::string & arm_name)
{
  Eigen::Matrix<double,7,1> desired_torque;
  desired_torque.setZero();
  if (cur_time_ >= init_time_ + 6.0)
  { 
    if (!force_control_init_)
    {
      estimated_ext_force_init_ = estimated_ext_force_;
      mode_init_time_ = cur_time_;
      x_mode_init_ = arm.transform_;
      force_control_init_ = true;
    }

    double traj_duration = 5.0;

    Eigen::Vector6d f_star;

    // Position control y, z
    x_target_.translation() << 0.5561, 0.0, 0.5902;
    x_target_.linear() << 0.664068,    0.747251,  -0.0250978,
                          0.746716,   -0.664542,  -0.0282643,
                          -0.037799, 2.84703e-05,   -0.999285;

    for (int i = 0; i < 3; i++)
    {
        x_desired_.translation()(i) = dyros_math::cubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
        x_dot_desired_(i) = dyros_math::cubicDot(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
    }
    
    x_desired_.linear() = dyros_math::rotationCubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.linear(), x_target_.linear()); 
    x_dot_desired_.segment(3,3) = dyros_math::rotationCubicDot(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.linear(), x_target_.linear()); 

    Eigen::Vector6d x_error;
    x_error.setZero();
    Eigen::Vector6d x_dot_error;
    x_dot_error.setZero();

    x_error.segment(0,3) = x_desired_.translation() - arm.position_;
    x_error.segment(3,3) = -dyros_math::getPhi(arm.rotation_, x_desired_.linear());
    x_dot_error.segment(0,3)= x_dot_desired_.segment(0,3) - arm.xd_.segment(0,3);
    x_dot_error.segment(3,3)= x_dot_desired_.segment(3,3) - arm.xd_.segment(3,3);

    f_star = kp_task_*x_error +kv_task_*x_dot_error;

    // Force control z
    f_d_z_ = cubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, estimated_ext_force_init_(2), -5.0, 0.0, 0.0);

    f_star(2) = 0.0;
    f_I_ = f_I_ + 1.0 * (f_d_z_ - estimated_ext_force_(2)) / 1000;
    

    Eigen::VectorXd F_d;
    F_d.resize(6);
    F_d.setZero();
    F_d(2) = f_d_z_ + f_I_;
    
    desired_torque = arm.jacobian_.transpose()*(arm.modified_lambda_matrix_*f_star + F_d) + arm.coriolis_;

    if (collision_state_)
      desired_torque.setZero();

    if (++ print_count_ > iter_per_print_)
    {
      print_count_ = 0;
    }
  }

  arm.setTorque(desired_torque);
  return true;
}

bool ResidualActionServer::setInitTarget(ros::Time time)
{
  q_target_ << 0, 0, 0, -90*DEG2RAD, 0, 90*DEG2RAD, 0;

  traj_duration_ = 5.0;
  traj_init_time_ = init_time_;

  for (int i = 0; i < 7; i++)
  {
    q_desired_(i) = dyros_math::cubic(time.toSec(), init_time_, init_time_ + traj_duration_, init_q_(i), q_target_(i), 0.0, 0.0);
    qd_desired_(i) = dyros_math::cubic(time.toSec(), init_time_, init_time_ + traj_duration_, init_qd_(i), q_target_(i), 0.0, 0.0);
  }     
}
bool ResidualActionServer::setRandomTarget(ros::Time time)
{
  if (cur_time_ >= traj_init_time_ + traj_duration_ + 1.0)
  {   
    if (next_traj_prepared_)
    {
      random_plan_ = random_plan_next_;
      cur_waypoint_ = 0;
      traj_init_time_ = cur_time_;//ros::Time::now().toSec();
      total_waypoints_ = random_plan_.trajectory_.joint_trajectory.points.size();
      traj_duration_ = random_plan_.trajectory_.joint_trajectory.points[total_waypoints_-1].time_from_start.toSec();

      next_traj_prepared_ = false; 
      std::cout<<"New Trajectory!"<< std::endl;
      std::cout<<"Total Waypoint: "<< total_waypoints_ << std::endl;
      std::cout << "Init Pose: " << q_init_plan_[0] << " " << q_init_plan_[1] << " " << q_init_plan_[2] << " " << q_init_plan_[3] << " " << q_init_plan_[4] << " " << q_init_plan_[5] << " " << q_init_plan_[6] << std::endl;
      std::cout << "Target Pose: " << q_target_plan_[0] << " " << q_target_plan_[1] << " " << q_target_plan_[2] << " " << q_target_plan_[3] << " " << q_target_plan_[4] << " " << q_target_plan_[5] << " " << q_target_plan_[6] << std::endl;
      std::cout<<"Trajetory Duration: " << traj_duration_ << std::endl << std::endl;
    }
    if (!init_traj_started_)
      init_traj_started_ = true;
  }
  else if (cur_time_ >= traj_init_time_ + traj_duration_)
  {
      // Rest
      if (init_traj_started_ && !next_traj_prepared_)
        current_traj_finished_ = true;
  }
  else if (cur_time_ >= traj_init_time_ + random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_+1].time_from_start.toSec())
  {
      if (cur_waypoint_ < total_waypoints_-2)
          cur_waypoint_++;
  }
  
  if (init_traj_started_)
  {
    double way_point_start_time = traj_init_time_ + random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_].time_from_start.toSec();
    double way_point_end_time = traj_init_time_ + random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_+1].time_from_start.toSec();

    std::vector<Eigen::Vector3d> traj;
    traj.resize(num_dof_);

    for (int i = 0; i < num_dof_; i++)
    {
        double init_q = random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_].positions[i];
        double init_q_dot = random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_].velocities[i];
        double init_q_ddot = random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_].accelerations[i];
        double target_q = random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_+1].positions[i];
        double target_q_dot = random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_+1].velocities[i];
        double target_q_ddot = random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_+1].accelerations[i];

        traj[i] = dyros_math::quinticSpline(cur_time_, way_point_start_time, way_point_end_time, init_q, init_q_dot, init_q_ddot, target_q, target_q_dot, target_q_ddot);

        q_desired_(i) = traj[i](0);
        qd_desired_(i) = traj[i](1);
        qdd_desired_(i) = traj[i](2);
    }
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
