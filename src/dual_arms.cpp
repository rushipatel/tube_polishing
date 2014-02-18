#include "dualArms.h"


/*! \brief Constructor. Subscribes to various services.
 *
 */
dualArms::dualArms(ros::NodeHandle& rh)
{
    //rh_ = rh;
    traj_client_r_  = new TrajClient("r_arm_controller/joint_trajectory_action", true);
    traj_client_l_  = new TrajClient("l_arm_controller/joint_trajectory_action", true);
    while(!traj_client_r_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the right_arm_controller/joint_trajectory_action server");
    }
    while(!traj_client_l_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the left_arm_controller/joint_trajectory_action server");
    }
    ros::service::waitForService("pr2_right_arm_kinematics/get_ik_solver_info");
    ros::service::waitForService("pr2_right_arm_kinematics/get_constraint_aware_ik");
    ros::service::waitForService("pr2_left_arm_kinematics/get_ik_solver_info");
    ros::service::waitForService("pr2_left_arm_kinematics/get_constraint_aware_ik");
    ros::service::waitForService("trajectory_filter_unnormalizer/filter_trajectory");
    ik_client_r_ = rh.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("pr2_right_arm_kinematics/get_constraint_aware_ik");
    query_client_r_ = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");
    ik_client_l_ = rh.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("pr2_left_arm_kinematics/get_constraint_aware_ik");
    query_client_l_ = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_left_arm_kinematics/get_ik_solver_info");
    filter_trajectory_client_ = rh.serviceClient<arm_navigation_msgs::FilterJointTrajectory>("trajectory_filter_unnormalizer/filter_trajectory");
}

/*! \brief Gets current joint angle from pr2 controller topics for initial IK seeds.
 *
 */
void dualArms::get_current_right_joint_angles(double current_angles[7])
{
    int i;

    //get a single message from the topic 'r_arm_controller/state'
    pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state_msg =
      ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>
      ("r_arm_controller/state");

    //extract the joint angles from it
    for(i=0; i<7; i++){
      current_angles[i] = state_msg->actual.positions[i];
    }
}

/*! \brief Gets current joint angle from pr2 controller topics for initial IK seeds.
 *
 */
void dualArms::get_current_left_joint_angles(double current_angles[7])
{
  int i;

  //get a single message from the topic 'l_arm_controller/state'
  pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state_msg =
    ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>
    ("l_arm_controller/state");

  //extract the joint angles from it
  for(i=0; i<7; i++){
    current_angles[i] = state_msg->actual.positions[i];
  }
}

/*! \brief Generates trajectory by calling IK service and stores output in double linear vector.
 *
 *  Returns false if IK fails at any trajectory point. This function must be called before calling executeTrajectory() function.
 */
bool dualArms::genTrajectory()
{
    right_joint_traj_.clear();
    left_joint_traj_.clear();
    if(!call_right_arm_gpik_(right_joint_traj_))
        return 0;
    if(!call_left_arm_gpik_(left_joint_traj_))
        return 0;
return 1;
}

/*! \brief Generates trajectory by calling IK service and stores output in double linear vector.
 *
 *  Returns false if IK fails at any trajectory point. This function must be called before calling executeTrajectory() function.
 */
bool dualArms::genTrajectory(std::vector<double> &rightJointTraj, std::vector<double> &lefttJointTraj)
{
    rightJointTraj.clear();
    lefttJointTraj.clear();
    if(!call_right_arm_gpik_(rightJointTraj))
        return 0;
    if(!call_left_arm_gpik_(lefttJointTraj))
        return 0;
return 1;
}


/*! \brief Generates Left trajectory by calling IK service and stores output in double linear vector.
 *
 *  Returns false if IK fails at any trajectory point.
 */
bool dualArms::genLeftTrajectory(std::vector<double> &jointTrajectory)
{
    jointTrajectory.clear();
    if(!call_left_arm_gpik_(jointTrajectory))
        return 0;
return 1;
}

/*! \brief Generates Right trajectory by calling IK service and stores output in double linear vector.
 *
 *  Returns false if IK fails at any trajectory point.
 */
bool dualArms::genRightTrajectory(std::vector<double> &jointTrajectory)
{
    jointTrajectory.clear();
    if(!call_right_arm_gpik_(jointTrajectory))
        return 0;
return 1;
}

/*! \brief Calls unnormalizer filter to remove any wrap arounds in generated trajectory.
 *
 *  
 */
void dualArms::call_right_joints_unnormalizer_()
{
    arm_navigation_msgs::FilterJointTrajectory::Request req;
    arm_navigation_msgs::FilterJointTrajectory::Response res;
    req.allowed_time = ros::Duration(60);

    req.start_state.joint_state.name.push_back("r_shoulder_pan_joint");
    req.start_state.joint_state.name.push_back("r_shoulder_lift_joint");
    req.start_state.joint_state.name.push_back("r_upper_arm_roll_joint");
    req.start_state.joint_state.name.push_back("r_elbow_flex_joint");
    req.start_state.joint_state.name.push_back("r_forearm_roll_joint");
    req.start_state.joint_state.name.push_back("r_wrist_flex_joint");
    req.start_state.joint_state.name.push_back("r_wrist_roll_joint");
    req.start_state.joint_state.position.resize(7);

    for(unsigned int i=0; i<7; i++)
        req.start_state.joint_state.position[i] = right_goal_.trajectory.points[1].positions[i];


    req.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    req.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    req.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    req.trajectory.joint_names.push_back("r_elbow_flex_joint");
    req.trajectory.joint_names.push_back("r_forearm_roll_joint");
    req.trajectory.joint_names.push_back("r_wrist_flex_joint");
    req.trajectory.joint_names.push_back("r_wrist_roll_joint");
    req.trajectory.points.resize(right_goal_.trajectory.points.size());

    for(unsigned int i=0; i<right_goal_.trajectory.points.size(); i++)
        req.trajectory.points[i].positions.resize(7);

    for(unsigned int i=0; i<right_goal_.trajectory.points.size(); i++)
    {
        for(int j=0; j<7; j++)
            req.trajectory.points[i].positions[j] = right_goal_.trajectory.points[i].positions[j];
    }

    if(filter_trajectory_client_.call(req,res))
    {
        if(res.error_code.val == res.error_code.SUCCESS)
        {
            right_goal_.trajectory = res.trajectory;
        }
        else
        {
            ROS_ERROR("Requested right trajectory was not filtered. Error code: %d",res.error_code.val);
            ros::shutdown();
            exit(-1);
        }
    }
    else
    {
        ROS_ERROR("Service call to right filter trajectory failed %s",filter_trajectory_client_.getService().c_str());
        ros::shutdown();
        exit(-1);
    }
}

/*! \brief Calls unnormalizer filter to remove any wrap arounds in generated trajectory.
 *
 *  
 */
void dualArms::call_left_joints_unnormalizer_()
{
    arm_navigation_msgs::FilterJointTrajectory::Request req;
    arm_navigation_msgs::FilterJointTrajectory::Response res;
    req.allowed_time = ros::Duration(1);

    req.start_state.joint_state.name.push_back("l_shoulder_pan_joint");
    req.start_state.joint_state.name.push_back("l_shoulder_lift_joint");
    req.start_state.joint_state.name.push_back("l_upper_arm_roll_joint");
    req.start_state.joint_state.name.push_back("l_elbow_flex_joint");
    req.start_state.joint_state.name.push_back("l_forearm_roll_joint");
    req.start_state.joint_state.name.push_back("l_wrist_flex_joint");
    req.start_state.joint_state.name.push_back("l_wrist_roll_joint");
    req.start_state.joint_state.position.resize(7);

    for(unsigned int i=0; i<7; i++)
        req.start_state.joint_state.position[i] = left_goal_.trajectory.points[1].positions[i];


    req.trajectory.joint_names.push_back("l_shoulder_pan_joint");
    req.trajectory.joint_names.push_back("l_shoulder_lift_joint");
    req.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
    req.trajectory.joint_names.push_back("l_elbow_flex_joint");
    req.trajectory.joint_names.push_back("l_forearm_roll_joint");
    req.trajectory.joint_names.push_back("l_wrist_flex_joint");
    req.trajectory.joint_names.push_back("l_wrist_roll_joint");
    req.trajectory.points.resize(left_goal_.trajectory.points.size());

    for(unsigned int i=0; i<left_goal_.trajectory.points.size(); i++)
        req.trajectory.points[i].positions.resize(7);

    for(unsigned int i=0; i<left_goal_.trajectory.points.size(); i++)
    {
        for(int j=0; j<7; j++)
            req.trajectory.points[i].positions[j] = left_goal_.trajectory.points[i].positions[j];
    }

    if(filter_trajectory_client_.call(req,res))
    {
        if(res.error_code.val == res.error_code.SUCCESS)
        {
            left_goal_.trajectory = res.trajectory;
        }
        else
        {
            ROS_ERROR("Requested left trajectory was not filtered. Error code: %d",res.error_code.val);
            ros::shutdown();
            exit(-1);
        }
    }
    else
    {
        ROS_ERROR("Service call to left filter trajectory failed %s",filter_trajectory_client_.getService().c_str());
        ros::shutdown();
        exit(-1);
    }
}

/*! \brief Populates trajecty goal variables from double vector.
 *
 *  
 */
void dualArms::get_right_goal_()
{
    trajectory_msgs::JointTrajectoryPoint traj_point;

    traj_point.positions.resize(7);
    traj_point.velocities.resize(7);

    right_goal_.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    right_goal_.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    right_goal_.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    right_goal_.trajectory.joint_names.push_back("r_elbow_flex_joint");
    right_goal_.trajectory.joint_names.push_back("r_forearm_roll_joint");
    right_goal_.trajectory.joint_names.push_back("r_wrist_flex_joint");
    right_goal_.trajectory.joint_names.push_back("r_wrist_roll_joint");

    right_goal_.trajectory.points.resize(objPoseTraj.poses.size());

    for(int j=0; j<7; j++)
    {
        traj_point.positions[j] = left_joint_traj_[j];
        traj_point.velocities[j] = 0.0;
    }
    traj_point.time_from_start = ros::Duration(0.25);
    right_goal_.trajectory.points[0] = traj_point;

    for(unsigned int i=1; i<objPoseTraj.poses.size(); i++)
    {
        for(int j=0; j<7; j++)
        {
            traj_point.positions[j] = left_joint_traj_[(i*7)+j];
            traj_point.velocities[j] = 0.0;
        }
        right_goal_.trajectory.points[i] = traj_point;
    }
}

/*! \brief Populates trajecty goal variables from double vector.
 *
 *  
 */
void dualArms::get_left_goal_()
{
    trajectory_msgs::JointTrajectoryPoint traj_point;

    traj_point.positions.resize(7);
    traj_point.velocities.resize(7);

    left_goal_.trajectory.joint_names.push_back("l_shoulder_pan_joint");
    left_goal_.trajectory.joint_names.push_back("l_shoulder_lift_joint");
    left_goal_.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
    left_goal_.trajectory.joint_names.push_back("l_elbow_flex_joint");
    left_goal_.trajectory.joint_names.push_back("l_forearm_roll_joint");
    left_goal_.trajectory.joint_names.push_back("l_wrist_flex_joint");
    left_goal_.trajectory.joint_names.push_back("l_wrist_roll_joint");

    left_goal_.trajectory.points.resize(objPoseTraj.poses.size());

    for(int j=0; j<7; j++)
    {
        traj_point.positions[j] = left_joint_traj_[j];
        traj_point.velocities[j] = 0.0;
    }
    traj_point.time_from_start = ros::Duration(0.25);
    left_goal_.trajectory.points[0] = traj_point;

    for(unsigned int i=1; i<objPoseTraj.poses.size(); i++)
    {
        for(int j=0; j<7; j++)
        {
            traj_point.positions[j] = left_joint_traj_[(i*7)+j];
            traj_point.velocities[j] = 0.0;
        }
        left_goal_.trajectory.points[i] = traj_point;
    }
}

/*! \brief Synchronizes start times of both goal joint trajectory based on maximum joint move in both arms.
 *
 *  Note: MAX_JOINT_VEL=0.5 is defined in dualArms.h file.
 */
void dualArms::sync_start_times_(void)
{
    double max_right_joint_move = 0, max_left_joint_move = 0, max_joint_move=0;
    double time_from_start = 0.25;
    for(unsigned int i=1; i < objPoseTraj.poses.size(); i++)
    {
        max_right_joint_move = 0;
        for(int j=0; j<7; j++)
        {
            double joint_move = fabs(right_goal_.trajectory.points[i].positions[j] - right_goal_.trajectory.points[i-1].positions[j]);
            if(joint_move > max_right_joint_move) max_right_joint_move = joint_move;
        }

        max_left_joint_move = 0;
        for(int j=0; j<7; j++)
        {
            double joint_move = fabs(left_goal_.trajectory.points[i].positions[j] - left_goal_.trajectory.points[i-1].positions[j]);
            if(joint_move > max_left_joint_move) max_left_joint_move = joint_move;
        }

        if(max_right_joint_move>max_left_joint_move)
            max_joint_move = max_right_joint_move;
        else
            max_joint_move = max_left_joint_move;

        double seconds = max_joint_move/MAX_JOINT_VEL;
        if(seconds>6.0)
            ROS_WARN("max_joint_move: %0.3f, seconds: %0.3f at traj point %d; check wrap arounds in joints", max_joint_move, seconds, i);
        time_from_start += seconds;
        right_goal_.trajectory.points[i].time_from_start = ros::Duration(time_from_start);
        left_goal_.trajectory.points[i].time_from_start = ros::Duration(time_from_start);
    }
}

/*! \brief Executes joint trajectory for both arms.
 *
 *  Note: genTrajectory() function must be called before calling this function.
 */
bool dualArms::executeJointTrajectory()
{
    get_right_goal_();
    get_left_goal_();
    call_right_joints_unnormalizer_();
    call_left_joints_unnormalizer_();
    sync_start_times_();

    ros::Time time_to_start = ros::Time::now()+ros::Duration(1.0);
    right_goal_.trajectory.header.stamp = time_to_start; //ros::Time::now()+ros::Duration(1.0);
    left_goal_.trajectory.header.stamp = time_to_start; //ros::Time::now()+ros::Duration(1.0);
    traj_client_r_->sendGoal(right_goal_);
    traj_client_l_->sendGoal(left_goal_);
    traj_client_r_->waitForResult();
    traj_client_l_->waitForResult();
    return(1);
}

/*! \brief Executes joint trajectory for both arms.
 *
 *  Note: genTrajectory() function must be called before calling this function.
 */
bool dualArms::executeJointTrajectory(std::vector<double> &qRight, std::vector<double> &qLeft)
{
    right_joint_traj_ = qRight;
    left_joint_traj_ = qLeft;
    get_right_goal_();
    get_left_goal_();
    call_right_joints_unnormalizer_();
    call_left_joints_unnormalizer_();
    sync_start_times_();

    ros::Time time_to_start = ros::Time::now()+ros::Duration(1.0);
    right_goal_.trajectory.header.stamp = time_to_start; //ros::Time::now()+ros::Duration(1.0);
    left_goal_.trajectory.header.stamp = time_to_start; //ros::Time::now()+ros::Duration(1.0);
    traj_client_r_->sendGoal(right_goal_);
    traj_client_l_->sendGoal(left_goal_);
    traj_client_r_->waitForResult();
    traj_client_l_->waitForResult();
    return(1);
}

/*! \brief Simple move arm function to move individual arm for given pose.
 *
 *
 */
bool dualArms::moveRightArm(geometry_msgs::Pose pose)
{
    double crnt_joints[7];
    pr2_controllers_msgs::JointTrajectoryGoal traj_goal;
    trajectory_msgs::JointTrajectoryPoint goal;
    kinematics_msgs::GetConstraintAwarePositionIK::Request  ik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;

    ik_req.timeout = ros::Duration(5.0);
    ik_req.ik_request.ik_link_name = "r_wrist_roll_link";
    ik_req.ik_request.ik_seed_state.joint_state.name.push_back("r_shoulder_pan_joint");
    ik_req.ik_request.ik_seed_state.joint_state.name.push_back("r_shoulder_lift_joint");
    ik_req.ik_request.ik_seed_state.joint_state.name.push_back("r_upper_arm_roll_joint");
    ik_req.ik_request.ik_seed_state.joint_state.name.push_back("r_elbow_flex_joint");
    ik_req.ik_request.ik_seed_state.joint_state.name.push_back("r_forearm_roll_joint");
    ik_req.ik_request.ik_seed_state.joint_state.name.push_back("r_wrist_flex_joint");
    ik_req.ik_request.ik_seed_state.joint_state.name.push_back("r_wrist_roll_joint");
    ik_req.ik_request.ik_seed_state.joint_state.position.resize(7);
    dualArms::get_current_right_joint_angles(crnt_joints);
    for(unsigned int i=0; i<7; i++)
        ik_req.ik_request.ik_seed_state.joint_state.position[i] = crnt_joints[i];
    ik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
    ik_req.ik_request.pose_stamped.pose = pose;

    if(ik_client_r_.call(ik_req, ik_res))
    {
      if(ik_res.error_code.val == ik_res.error_code.SUCCESS)
      {
          traj_goal.trajectory.points.resize(1);
          traj_goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
          traj_goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
          traj_goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
          traj_goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
          traj_goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
          traj_goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
          traj_goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
          goal.positions.resize(7);
          goal.velocities.resize(7);
        for(int j=0; j<7; j++)
        {
            goal.positions[j] = ik_res.solution.joint_state.position[j];
            goal.velocities[j] = 0.0;
        }
        goal.time_from_start = ros::Duration(0.0);
        traj_goal.trajectory.points[0] = goal;
        ros::Time time_to_start = ros::Time::now()+ros::Duration(0.1);
        traj_goal.trajectory.header.stamp = time_to_start;
        traj_client_r_->sendGoalAndWait(traj_goal);
      }
      else
      {
        ROS_ERROR("Right arm Inverse kinematics failed for given pose.");
        return 0;
      }
    }
    else
    {
      ROS_ERROR("Right arm Inverse kinematics service call failed.");
      return 0;
    }
    return 1;
}

/*! \brief Simple move arm function to move individual arm for given pose.
 *
 *
 */
bool dualArms::moveLeftArm(geometry_msgs::Pose pose)
{
    double crnt_joints[7];
    pr2_controllers_msgs::JointTrajectoryGoal traj_goal;
    trajectory_msgs::JointTrajectoryPoint goal;
    kinematics_msgs::GetConstraintAwarePositionIK::Request  ik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;

    ik_req.timeout = ros::Duration(5.0);
    ik_req.ik_request.ik_link_name = "l_wrist_roll_link";
    ik_req.ik_request.ik_seed_state.joint_state.name.push_back("l_shoulder_pan_joint");
    ik_req.ik_request.ik_seed_state.joint_state.name.push_back("l_shoulder_lift_joint");
    ik_req.ik_request.ik_seed_state.joint_state.name.push_back("l_upper_arm_roll_joint");
    ik_req.ik_request.ik_seed_state.joint_state.name.push_back("l_elbow_flex_joint");
    ik_req.ik_request.ik_seed_state.joint_state.name.push_back("l_forearm_roll_joint");
    ik_req.ik_request.ik_seed_state.joint_state.name.push_back("l_wrist_flex_joint");
    ik_req.ik_request.ik_seed_state.joint_state.name.push_back("l_wrist_roll_joint");
    ik_req.ik_request.ik_seed_state.joint_state.position.resize(7);
    get_current_left_joint_angles(crnt_joints);
    for(unsigned int i=0; i<7; i++)
        ik_req.ik_request.ik_seed_state.joint_state.position[i] = crnt_joints[i];
    ik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
    ik_req.ik_request.pose_stamped.pose = pose;

    if(ik_client_l_.call(ik_req, ik_res))
    {
      if(ik_res.error_code.val == ik_res.error_code.SUCCESS)
      {
          traj_goal.trajectory.points.resize(1);
          traj_goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
          traj_goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
          traj_goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
          traj_goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
          traj_goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
          traj_goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
          traj_goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
          goal.positions.resize(7);
          goal.velocities.resize(7);
        for(int j=0; j<7; j++)
        {
            goal.positions[j] = ik_res.solution.joint_state.position[j];
            goal.velocities[j] = 0.0;
        }
        goal.time_from_start = ros::Duration(0.0);
        traj_goal.trajectory.points[0] = goal;
        ros::Time time_to_start = ros::Time::now()+ros::Duration(0.1);
        traj_goal.trajectory.header.stamp = time_to_start;
        traj_client_l_->sendGoalAndWait(traj_goal);
      }
      else
      {
        ROS_ERROR("Left arm Inverse kinematics failed for given pose.");
        return 0;
      }
    }
    else
    {
      ROS_ERROR("Left arm Inverse kinematics service call failed.");
      return 0;
    }
    return 1;
}

/*! \brief Calls IK service.
 *
 */
bool dualArms::call_right_arm_gpik_(std::vector<double> &right_joint_trajectory)
{
    kinematics_msgs::GetKinematicSolverInfo::Request request;
    kinematics_msgs::GetKinematicSolverInfo::Response response;
    double last_right_joints[7];
    geometry_msgs::Pose pose;
    tf::Transform tf_base_wrist, tf_base_obj;

    if(query_client_r_.call(request,response))
    {
      for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
      {
        ROS_DEBUG("right arm Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
      }
    }
    else
    {
      ROS_ERROR("Could not call right arm query service");
      ros::shutdown();
      exit(-1);
    }

    kinematics_msgs::GetConstraintAwarePositionIK::Request  gpik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response gpik_res;
    gpik_req.timeout = ros::Duration(5.0);
    gpik_req.ik_request.ik_link_name = "r_wrist_roll_link";

    gpik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
    gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
    gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
    dualArms::get_current_right_joint_angles(last_right_joints);

    for(unsigned int i=0; i<objPoseTraj.poses.size(); i++)
    {
        tf_base_obj = pose2tf(objPoseTraj.poses[i]);
        tf_base_wrist = tf_base_obj*rightWristOffset;
        pose = tf2pose(tf_base_wrist);
        for(int k=0; k<7; k++)
            gpik_req.ik_request.ik_seed_state.joint_state.position[k] = last_right_joints[k];
        gpik_req.ik_request.pose_stamped.pose = pose;
        if(ik_client_r_.call(gpik_req, gpik_res))
        {
          if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
          {
            for(unsigned int j=0; j<gpik_res.solution.joint_state.name.size(); j++)
            {
              //ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
              right_joint_trajectory.push_back(gpik_res.solution.joint_state.position[j]);
            }
            for( int l=0; l<7; l++)
                last_right_joints[l] = gpik_res.solution.joint_state.position[l];
          }
          else
          {
            ROS_DEBUG("right arm Inverse kinematics failed at pose no. %d",i);
            return 0;
          }
        }
        else
        {
          ROS_ERROR("right arm Inverse kinematics service call failed at pose no. %d",i);
          return 0;
        }
    }
    return 1;
}

/*! \brief Calls IK service.
 *
 */
bool dualArms::call_left_arm_gpik_(std::vector<double> &left_joint_trajectory)
{
    kinematics_msgs::GetKinematicSolverInfo::Request request;
    kinematics_msgs::GetKinematicSolverInfo::Response response;
    double last_left_joints[7];
    geometry_msgs::Pose pose;
    tf::Transform tf_base_wrist, tf_base_obj;

    if(query_client_l_.call(request,response))
    {
      for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
      {
        ROS_DEBUG("left_arm Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
      }
    }
    else
    {
      ROS_ERROR("Could not call left_arm query service");
      ros::shutdown();
      exit(-1);
    }

    kinematics_msgs::GetConstraintAwarePositionIK::Request  gpik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response gpik_res;
    gpik_req.timeout = ros::Duration(5.0);
    gpik_req.ik_request.ik_link_name = "l_wrist_roll_link";

    gpik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
    gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
    gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
    //double joints[7];
    get_current_left_joint_angles(last_left_joints);

    for(unsigned int i=0; i<objPoseTraj.poses.size(); i++)
    {
        tf_base_obj = pose2tf(objPoseTraj.poses[i]);
        tf_base_wrist = tf_base_obj*leftWristOffset;
        pose = tf2pose(tf_base_wrist);
        for(int k=0; k<7; k++)
            gpik_req.ik_request.ik_seed_state.joint_state.position[k] = last_left_joints[k];
        gpik_req.ik_request.pose_stamped.pose = pose;
        if(ik_client_l_.call(gpik_req, gpik_res))
        {
          if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
          {
            for(unsigned int j=0; j<gpik_res.solution.joint_state.name.size(); j++)
            {
              //ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
              left_joint_trajectory.push_back(gpik_res.solution.joint_state.position[j]);
            }
            for( int l=0; l<7; l++)
                last_left_joints[l] = gpik_res.solution.joint_state.position[l];
          }
          else
          {
            ROS_DEBUG("left_arm Inverse kinematics failed at pose no. %d",i);
            return 0;
          }
        }
        else
        {
          ROS_ERROR("left_arm Inverse kinematics service call failed at pose no. %d",i);
          return 0;
        }
    }
    return 1;
}
