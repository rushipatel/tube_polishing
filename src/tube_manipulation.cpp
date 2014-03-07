#include "tubeManipulation.h"


/*! \brief Constructor. Subscribes to various services.
 *
 */
TubeManipulation::Arms::Arms(ros::NodeHandlePtr rh)
{
    _traj_client_r  = new TrajClient("r_arm_controller/joint_trajectory_action", true);
    _traj_client_l  = new TrajClient("l_arm_controller/joint_trajectory_action", true);
    while(!_traj_client_r->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the right_arm_controller/joint_trajectory_action server");
    }
    while(!_traj_client_l->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the left_arm_controller/joint_trajectory_action server");
    }
    ros::service::waitForService("pr2_right_arm_kinematics/get_ik_solver_info");
    ros::service::waitForService("pr2_right_arm_kinematics/get_constraint_aware_ik");
    ros::service::waitForService("pr2_right_arm_kinematics/get_ik");
    ros::service::waitForService("pr2_left_arm_kinematics/get_ik_solver_info");
    ros::service::waitForService("pr2_left_arm_kinematics/get_constraint_aware_ik");
    ros::service::waitForService("pr2_left_arm_kinematics/get_ik");
    ros::service::waitForService("trajectory_filter_unnormalizer/filter_trajectory");

    _set_pln_scn_client = rh->serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
    _fk_client_r = rh->serviceClient<kinematics_msgs::GetPositionFK>("pr2_right_arm_kinematics/get_fk");
    _fk_client_l = rh->serviceClient<kinematics_msgs::GetPositionFK>("pr2_left_arm_kinematics/get_fk");
    _ik_client_r = rh->serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("pr2_right_arm_kinematics/get_constraint_aware_ik");
    _smpl_ik_client_r = rh->serviceClient<kinematics_msgs::GetPositionIK>("pr2_right_arm_kinematics/get_ik");
    _query_client_r = rh->serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");
    _ik_client_l = rh->serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("pr2_left_arm_kinematics/get_constraint_aware_ik");
    _smpl_ik_client_l = rh->serviceClient<kinematics_msgs::GetPositionIK>("pr2_left_arm_kinematics/get_ik");
    _query_client_l = rh->serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_left_arm_kinematics/get_ik_solver_info");
    _filter_trajectory_client =
            rh->serviceClient<arm_navigation_msgs::FilterJointTrajectory>("trajectory_filter_unnormalizer/filter_trajectory");

    arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
    if(!_set_pln_scn_client.call(planning_scene_req, planning_scene_res))
        ROS_ERROR("Can't set planning scene");

}

void TubeManipulation::Arms::setObjPoseTrajectory(geometry_msgs::PoseArray &pose_array)
{
    _obj_pose_traj = pose_array;
}

void TubeManipulation::Arms::setWristOffset(tf::Transform &right_offset, tf::Transform &left_offset)
{
    _right_wrist_offset = right_offset;
    _left_wrist_offset = left_offset;
}

void TubeManipulation::Arms::setWristOffset(geometry_msgs::Pose &right_offset, geometry_msgs::Pose &left_offset)
{
    _right_wrist_offset = pose2tf(right_offset);
    _left_wrist_offset = pose2tf(left_offset);
}

/*! \brief Gets current joint angles from pr2 controller topics for initial IK seeds.
 *
 */
void TubeManipulation::Arms::_get_right_joints(std::vector<double> &joint_state)
{
    //get a single message from the topic 'r_arm_controller/state'
    pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state_msg =
      ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>
      ("r_arm_controller/state");

    joint_state.resize(7);
    //extract the joint angles from it
    for(unsigned int i=0; i<7; i++)
      joint_state[i] = state_msg->actual.positions[i];
}

/*! \brief Gets current joint angles from pr2 controller topics for initial IK seeds.
 *
 */
void TubeManipulation::Arms::_get_left_joints(std::vector<double> &joint_state)
{
    //get a single message from the topic 'l_arm_controller/state'
    pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state_msg =
      ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>
      ("l_arm_controller/state");
    joint_state.clear();
    joint_state.resize(7);
    //extract the joint angles from message
    for(unsigned int i=0; i<7; i++)
      joint_state[i] = state_msg->actual.positions[i];
}

void TubeManipulation::Arms::_get_default_right_joints(std::vector<double> &joint_state)
{
    joint_state.resize(7);
    joint_state[0] = 0;
    joint_state[1] = 0;
    joint_state[2] = 0;
    joint_state[3] = 0;
    joint_state[4] = 0;
    joint_state[5] = 0;
    joint_state[6] = 0;
}

void TubeManipulation::Arms::_get_default_left_joints(std::vector<double> &joint_state)
{
    joint_state.resize(7);
    joint_state[0] = 0;
    joint_state[1] = 0;
    joint_state[2] = 0;
    joint_state[3] = 0;
    joint_state[4] = 0;
    joint_state[5] = 0;
    joint_state[6] = 0;
}

geometry_msgs::Pose TubeManipulation::Arms::_get_right_fk(std::vector<double> &joints)
{
    kinematics_msgs::GetPositionFK::Request req;
    kinematics_msgs::GetPositionFK::Response res;

    req.fk_link_names.push_back("r_wrist_roll_link");
    req.header.frame_id = "\base_link";
    req.header.stamp = ros::Time::now();

    req.robot_state.joint_state.header.frame_id = "\base_link";
    req.robot_state.joint_state.header.stamp = ros::Time::now();

    req.robot_state.joint_state.name.resize(joints.size());
    req.robot_state.joint_state.name.push_back("r_shoulder_pan_joint");
    req.robot_state.joint_state.name.push_back("r_shoulder_lift_joint");
    req.robot_state.joint_state.name.push_back("r_upper_arm_roll_joint");
    req.robot_state.joint_state.name.push_back("r_elbow_flex_joint");
    req.robot_state.joint_state.name.push_back("r_forearm_roll_joint");
    req.robot_state.joint_state.name.push_back("r_wrist_flex_joint");
    req.robot_state.joint_state.name.push_back("r_wrist_roll_joint");

    req.robot_state.joint_state.position.resize(joints.size());
    for(int i=0; i<joints.size(); i++)
        req.robot_state.joint_state.position[i] = joints[i];

    geometry_msgs::PoseStamped pose_stamped;
    if(_fk_client_r.call(req,res))
        pose_stamped = res.pose_stamped[0];
    else
        ROS_ERROR("Right FK call failed");
    return pose_stamped.pose;
}

geometry_msgs::Pose TubeManipulation::Arms::_get_left_fk(std::vector<double> &joints)
{
    kinematics_msgs::GetPositionFK::Request req;
    kinematics_msgs::GetPositionFK::Response res;

    req.fk_link_names.push_back("l_wrist_roll_link");
    req.header.frame_id = "\base_link";
    req.header.stamp = ros::Time::now();

    req.robot_state.joint_state.header.frame_id = "\base_link";
    req.robot_state.joint_state.header.stamp = ros::Time::now();

    req.robot_state.joint_state.name.resize(joints.size());
    req.robot_state.joint_state.name.push_back("l_shoulder_pan_joint");
    req.robot_state.joint_state.name.push_back("l_shoulder_lift_joint");
    req.robot_state.joint_state.name.push_back("l_upper_arm_roll_joint");
    req.robot_state.joint_state.name.push_back("l_elbow_flex_joint");
    req.robot_state.joint_state.name.push_back("l_forearm_roll_joint");
    req.robot_state.joint_state.name.push_back("l_wrist_flex_joint");
    req.robot_state.joint_state.name.push_back("l_wrist_roll_joint");

    req.robot_state.joint_state.position.resize(joints.size());
    for(int i=0; i<joints.size(); i++)
        req.robot_state.joint_state.position[i] = joints[i];

    geometry_msgs::PoseStamped pose_stamped;
    if(_fk_client_l.call(req,res))
        pose_stamped = res.pose_stamped[0];
    else
        ROS_ERROR("Left FK call failed");
    return pose_stamped.pose;
}

bool TubeManipulation::Arms::_gen_trarajectory(std::vector<double> &right_joint_traj, std::vector<double> &left_joint_traj)
{
    _right_joint_traj.clear();
    _left_joint_traj.clear();
    std::vector<double> right_joints(7), left_joints(7);
    sensor_msgs::JointState right_joint_state, left_joint_state;

    _get_default_right_joints(right_joints);
    _get_default_left_joints(left_joints);

    geometry_msgs::Pose right_pose, left_pose;
    tf::Transform tf_right_wrist, tf_left_wrist, tf_obj;
    if(_obj_pose_traj.poses.empty())
    {
        ROS_ERROR("dualArms - Object trajectory is empty");
        return false;
    }

    for(unsigned int i=0; i<_obj_pose_traj.poses.size(); i++)
    {
        tf_obj = pose2tf(_obj_pose_traj.poses[i]);

        tf_right_wrist = tf_obj*_right_wrist_offset;
        right_pose = tf2pose(tf_right_wrist);

        tf_left_wrist = tf_obj*_left_wrist_offset;
        left_pose = tf2pose(tf_left_wrist);

        if( _get_right_arm_ik(right_pose, right_joint_state, right_joints) &&
                 _get_left_arm_ik(left_pose, left_joint_state, left_joints) )
        {
            if(right_joint_state.position.size()==7 && left_joint_state.position.size()==7)
            {
                for(unsigned int i=0; i<right_joint_state.position.size(); i++)
                {
                    right_joint_traj.push_back(right_joint_state.position[i]);
                    left_joint_traj.push_back(left_joint_state.position[i]);

                    right_joints[i] = right_joint_state.position[i];
                    left_joints[i] = left_joint_state.position[i];
                }
            }
            else
                ROS_ERROR("dualArms - number of joints of IK results are not the same");
        }
        else
            return false;
    }

    return true;
}

/*! \brief Generates trajectory by calling IK service and stores output in double linear vector.
 *
 *  Returns false if IK fails at any trajectory point. This function must be called before calling executeTrajectory() function.
 */
bool TubeManipulation::Arms::genTrajectory()
{
    _right_joint_traj.clear();
    _left_joint_traj.clear();
    if(!_gen_trarajectory(_right_joint_traj, _left_joint_traj))
        return false;
    return true;
}

/*! \brief Generates trajectory by calling IK service and stores output in double linear vector.
 *
 *  Returns false if IK fails at any trajectory point. This function must be called before calling executeTrajectory() function.
 */
bool TubeManipulation::Arms::genTrajectory(std::vector<double> &rightJointTraj, std::vector<double> &lefttJointTraj)
{
    rightJointTraj.clear();
    lefttJointTraj.clear();
    if(!_gen_trarajectory(rightJointTraj, lefttJointTraj))
        return false;
    return true;
}

/*! \brief Calls unnormalizer filter to remove any wrap arounds in generated trajectory.
 *
 *  
 */
void TubeManipulation::Arms::_call_right_joints_unnormalizer(void)
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
        req.start_state.joint_state.position[i] = _right_goal.trajectory.points[1].positions[i];


    req.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    req.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    req.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    req.trajectory.joint_names.push_back("r_elbow_flex_joint");
    req.trajectory.joint_names.push_back("r_forearm_roll_joint");
    req.trajectory.joint_names.push_back("r_wrist_flex_joint");
    req.trajectory.joint_names.push_back("r_wrist_roll_joint");
    req.trajectory.points.resize(_right_goal.trajectory.points.size());

    for(unsigned int i=0; i<_right_goal.trajectory.points.size(); i++)
        req.trajectory.points[i].positions.resize(7);

    for(unsigned int i=0; i<_right_goal.trajectory.points.size(); i++)
    {
        for(int j=0; j<7; j++)
            req.trajectory.points[i].positions[j] = _right_goal.trajectory.points[i].positions[j];
    }

    if(_filter_trajectory_client.call(req,res))
    {
        if(res.error_code.val == res.error_code.SUCCESS)
        {
            _right_goal.trajectory = res.trajectory;
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
        ROS_ERROR("Service call to right filter trajectory failed %s",_filter_trajectory_client.getService().c_str());
        ros::shutdown();
        exit(-1);
    }
}

/*! \brief Calls unnormalizer filter to remove any wrap arounds in generated trajectory.
 *
 *  
 */
void TubeManipulation::Arms::_call_left_joints_unnormalizer()
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
        req.start_state.joint_state.position[i] = _left_goal.trajectory.points[1].positions[i];


    req.trajectory.joint_names.push_back("l_shoulder_pan_joint");
    req.trajectory.joint_names.push_back("l_shoulder_lift_joint");
    req.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
    req.trajectory.joint_names.push_back("l_elbow_flex_joint");
    req.trajectory.joint_names.push_back("l_forearm_roll_joint");
    req.trajectory.joint_names.push_back("l_wrist_flex_joint");
    req.trajectory.joint_names.push_back("l_wrist_roll_joint");
    req.trajectory.points.resize(_left_goal.trajectory.points.size());

    for(unsigned int i=0; i<_left_goal.trajectory.points.size(); i++)
        req.trajectory.points[i].positions.resize(7);

    for(unsigned int i=0; i<_left_goal.trajectory.points.size(); i++)
    {
        for(int j=0; j<7; j++)
            req.trajectory.points[i].positions[j] = _left_goal.trajectory.points[i].positions[j];
    }

    if(_filter_trajectory_client.call(req,res))
    {
        if(res.error_code.val == res.error_code.SUCCESS)
        {
            _left_goal.trajectory = res.trajectory;
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
        ROS_ERROR("Service call to left filter trajectory failed %s",_filter_trajectory_client.getService().c_str());
        ros::shutdown();
        exit(-1);
    }
}

/*! \brief Populates trajecty goal variables from double vector.
 *
 *  
 */
void TubeManipulation::Arms::_get_right_goal()
{
    trajectory_msgs::JointTrajectoryPoint traj_point;

    traj_point.positions.resize(7);
    traj_point.velocities.resize(7);

    _right_goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    _right_goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    _right_goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    _right_goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    _right_goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    _right_goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    _right_goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    _right_goal.trajectory.points.resize(_obj_pose_traj.poses.size());

    for(int j=0; j<7; j++)
    {
        traj_point.positions[j] = _left_joint_traj[j];
        traj_point.velocities[j] = 0.0;
    }
    traj_point.time_from_start = ros::Duration(0.25);
    _right_goal.trajectory.points[0] = traj_point;

    for(unsigned int i=1; i<_obj_pose_traj.poses.size(); i++)
    {
        for(int j=0; j<7; j++)
        {
            traj_point.positions[j] = _left_joint_traj[(i*7)+j];
            traj_point.velocities[j] = 0.0;
        }
        _right_goal.trajectory.points[i] = traj_point;
    }
}

/*! \brief Populates trajecty goal variables from double vector.
 *
 *  
 */
void TubeManipulation::Arms::_get_left_goal()
{
    trajectory_msgs::JointTrajectoryPoint traj_point;

    traj_point.positions.resize(7);
    traj_point.velocities.resize(7);

    _left_goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
    _left_goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
    _left_goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
    _left_goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
    _left_goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
    _left_goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
    _left_goal.trajectory.joint_names.push_back("l_wrist_roll_joint");

    _left_goal.trajectory.points.resize(_obj_pose_traj.poses.size());

    for(int j=0; j<7; j++)
    {
        traj_point.positions[j] = _left_joint_traj[j];
        traj_point.velocities[j] = 0.0;
    }
    traj_point.time_from_start = ros::Duration(0.25);
    _left_goal.trajectory.points[0] = traj_point;

    for(unsigned int i=1; i<_obj_pose_traj.poses.size(); i++)
    {
        for(int j=0; j<7; j++)
        {
            traj_point.positions[j] = _left_joint_traj[(i*7)+j];
            traj_point.velocities[j] = 0.0;
        }
        _left_goal.trajectory.points[i] = traj_point;
    }
}

/*! \brief Synchronizes start times of both goal joint trajectory based on maximum joint move in both arms.
 *
 *  Note: MAX_JOINT_VEL=0.5 is defined in dualArms.h file.
 */
void TubeManipulation::Arms::_sync_start_times(void)
{
    double max_right_joint_move = 0, max_left_joint_move = 0, max_joint_move=0;
    double time_from_start = 0.25;
    for(unsigned int i=1; i < _obj_pose_traj.poses.size(); i++)
    {
        max_right_joint_move = 0;
        for(int j=0; j<7; j++)
        {
            double joint_move = fabs(_right_goal.trajectory.points[i].positions[j] - _right_goal.trajectory.points[i-1].positions[j]);
            if(joint_move > max_right_joint_move) max_right_joint_move = joint_move;
        }

        max_left_joint_move = 0;
        for(int j=0; j<7; j++)
        {
            double joint_move = fabs(_left_goal.trajectory.points[i].positions[j] - _left_goal.trajectory.points[i-1].positions[j]);
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
        _right_goal.trajectory.points[i].time_from_start = ros::Duration(time_from_start);
        _left_goal.trajectory.points[i].time_from_start = ros::Duration(time_from_start);
    }
}

/*! \brief Executes joint trajectory for both arms.
 *
 *  Note: genTrajectory() function must be called before calling this function.
 */
bool TubeManipulation::Arms::executeJointTrajectory()
{
    _get_right_goal();
    _get_left_goal();
    _call_right_joints_unnormalizer();
    _call_left_joints_unnormalizer();
    _sync_start_times();

    ros::Time time_to_start = ros::Time::now()+ros::Duration(1.0);
    _right_goal.trajectory.header.stamp = time_to_start; //ros::Time::now()+ros::Duration(1.0);
    _left_goal.trajectory.header.stamp = time_to_start; //ros::Time::now()+ros::Duration(1.0);
    _traj_client_r->sendGoal(_right_goal);
    _traj_client_l->sendGoal(_left_goal);
    _traj_client_r->waitForResult();
    _traj_client_l->waitForResult();
    return(1);
}


/*! \brief Executes joint trajectory for both arms.
 *
 *  Note: genTrajectory() function must be called before calling this function.
 */
bool TubeManipulation::Arms::executeJointTrajectory(std::vector<double> &qRight, std::vector<double> &qLeft)
{
    _right_joint_traj = qRight;
    _left_joint_traj = qLeft;
    _get_right_goal();
    _get_left_goal();
    _call_right_joints_unnormalizer();
    _call_left_joints_unnormalizer();
    _sync_start_times();

    ros::Time time_to_start = ros::Time::now()+ros::Duration(1.0);
    _right_goal.trajectory.header.stamp = time_to_start; //ros::Time::now()+ros::Duration(1.0);
    _left_goal.trajectory.header.stamp = time_to_start; //ros::Time::now()+ros::Duration(1.0);
    _traj_client_r->sendGoal(_right_goal);
    _traj_client_l->sendGoal(_left_goal);
    _traj_client_r->waitForResult();
    _traj_client_l->waitForResult();
    return(1);
}

/*! \brief Simple move arm function to move individual arm for given pose.
 *
 *
 */
bool TubeManipulation::Arms::moveRightArm(geometry_msgs::Pose pose)
{
    std::vector<double> crnt_joints(7);
    pr2_controllers_msgs::JointTrajectoryGoal traj_goal;
    trajectory_msgs::JointTrajectoryPoint goal;
    sensor_msgs::JointState joint_state;

    _get_right_joints(crnt_joints);

    if(_get_right_arm_ik(pose, joint_state, crnt_joints))
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
        for(int i=0; i<7; i++)
        {
            goal.positions[i] = joint_state.position[i];
            goal.velocities[i] = 0.0;
        }
        goal.time_from_start = ros::Duration(0.0);
        traj_goal.trajectory.points[0] = goal;
        ros::Time time_to_start = ros::Time::now()+ros::Duration(0.1);
        traj_goal.trajectory.header.stamp = time_to_start;
        _traj_client_r->sendGoalAndWait(traj_goal);
        std::vector<double> joints(7);
        double err = 1;
        while(err>0.01)
        {
            _get_right_joints(joints);
            err = 0;
            for(int i=0; i<joints.size(); i++)
                err += joints[i] - goal.positions[i];
            ros::Duration(0.1).sleep();
        }
        ros::Duration(1).sleep();
    }
    else
        return 0;

    return 1;
}

/*! \brief Simple move arm function to move individual arm for given pose.
 *
 *
 */
bool TubeManipulation::Arms::moveLeftArm(geometry_msgs::Pose pose)
{
    std::vector<double> crnt_joints(7);
    pr2_controllers_msgs::JointTrajectoryGoal traj_goal;
    trajectory_msgs::JointTrajectoryPoint goal;
    sensor_msgs::JointState joint_state;

    _get_left_joints(crnt_joints);

    if(_get_left_arm_ik(pose, joint_state, crnt_joints))
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
        for(int i=0; i<7; i++)
        {
            goal.positions[i] = joint_state.position[i];
            goal.velocities[i] = 0.0;
        }
        goal.time_from_start = ros::Duration(0.0);
        traj_goal.trajectory.points[0] = goal;
        ros::Time time_to_start = ros::Time::now()+ros::Duration(0.1);
        traj_goal.trajectory.header.stamp = time_to_start;
        _traj_client_l->sendGoalAndWait(traj_goal);
        
        std::vector<double> joints(7);
        double err = 1;
        while(err>0.01)
        {
            _get_left_joints(joints);
            err = 0;
            for(int i=0; i<joints.size(); i++)
                err += joints[i] - goal.positions[i];
            ros::Duration(0.1).sleep();
        }
        ros::Duration(1).sleep();
    }
    else
        return 0;

    return 1;
}

/*! \brief Simple move arm function to move individual arm for given pose.
 *
 *
 */
bool TubeManipulation::Arms::simpleMoveRightArm(geometry_msgs::Pose pose)
{
    std::vector<double> crnt_joints(7);
    pr2_controllers_msgs::JointTrajectoryGoal traj_goal;
    trajectory_msgs::JointTrajectoryPoint goal;
    sensor_msgs::JointState joint_state;

    _get_right_joints(crnt_joints);

    if(_get_simple_right_arm_ik(pose, joint_state, crnt_joints))
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
            goal.positions[j] = joint_state.position[j];
            goal.velocities[j] = 0.0;
        }
        goal.time_from_start = ros::Duration(0.0);
        traj_goal.trajectory.points[0] = goal;
        ros::Time time_to_start = ros::Time::now()+ros::Duration(0.1);
        traj_goal.trajectory.header.stamp = time_to_start;
        _traj_client_r->sendGoalAndWait(traj_goal);
        std::vector<double> joints(7);
        double err = 1;
        while(err>0.01)
        {
            _get_right_joints(joints);
            err = 0;
            for(int i=0; i<joints.size(); i++)
            {  err += joints[i] - goal.positions[i];/* std::cout<<err<<" ";*/}
            ros::Duration(0.1).sleep();
        }
        ros::Duration(1).sleep();
    }
    else
        return 0;

    return 1;
}

/*! \brief Simple move arm function to move individual arm for given pose.
 *
 *
 */
bool TubeManipulation::Arms::simpleMoveLeftArm(geometry_msgs::Pose pose)
{
    std::vector<double> crnt_joints(7);
    pr2_controllers_msgs::JointTrajectoryGoal traj_goal;
    trajectory_msgs::JointTrajectoryPoint goal;
    sensor_msgs::JointState joint_state;

    _get_left_joints(crnt_joints);

    if(_get_simple_left_arm_ik(pose, joint_state, crnt_joints))
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
            goal.positions[j] = joint_state.position[j];
            goal.velocities[j] = 0.0;
        }
        goal.time_from_start = ros::Duration(0.0);
        traj_goal.trajectory.points[0] = goal;
        ros::Time time_to_start = ros::Time::now()+ros::Duration(0.1);
        traj_goal.trajectory.header.stamp = time_to_start;
        _traj_client_l->sendGoalAndWait(traj_goal);
        std::vector<double> joints(7);
        double err = 1;
        while(err>0.01)
        {
            _get_left_joints(joints);
            err = 0;
            for(int i=0; i<joints.size(); i++)
                err += joints[i] - goal.positions[i];
            ros::Duration(0.1).sleep();
        }
        ros::Duration(1).sleep();
    }
    else
        return 0;

    return 1;
}

///*! \brief Calls IK service.
// *
// */
//bool _call_right_arm_gpik(std::vector<double> &right_joint_trajectory)
//{
//    kinematics_msgs::GetKinematicSolverInfo::Request request;
//    kinematics_msgs::GetKinematicSolverInfo::Response response;
//    std::vector<double> last_joints;
//    geometry_msgs::Pose pose;
//    tf::Transform tf_base_wrist, tf_base_obj;

//    if(_query_client_r.call(request,response))
//    {
//      for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
//      {
//        ROS_DEBUG("right arm Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
//      }
//    }
//    else
//    {
//      ROS_ERROR("Could not call right arm query service");
//      ros::shutdown();
//      exit(-1);
//    }

//    kinematics_msgs::GetConstraintAwarePositionIK::Request  gpik_req;
//    kinematics_msgs::GetConstraintAwarePositionIK::Response gpik_res;
//    gpik_req.timeout = ros::Duration(5.0);
//    gpik_req.ik_request.ik_link_name = "r_wrist_roll_link";

//    gpik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
//    gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
//    gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
//    _get_right_joints(last_joints);

//    for(unsigned int i=0; i<_obj_pose_traj.poses.size(); i++)
//    {
//        tf_base_obj = pose2tf(_obj_pose_traj.poses[i]);
//        tf_base_wrist = tf_base_obj*_right_wrist_offset;
//        pose = tf2pose(tf_base_wrist);
//        for(int k=0; k<7; k++)
//            gpik_req.ik_request.ik_seed_state.joint_state.position[k] = last_joints[k];
//        gpik_req.ik_request.pose_stamped.pose = pose;
//        if(_ik_client_r.call(gpik_req, gpik_res))
//        {
//          if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
//          {
//            for(unsigned int j=0; j<gpik_res.solution.joint_state.name.size(); j++)
//            {
//              //ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
//              right_joint_trajectory.push_back(gpik_res.solution.joint_state.position[j]);
//            }
//            for( int l=0; l<7; l++)
//                last_joints[l] = gpik_res.solution.joint_state.position[l];
//          }
//          else
//          {
//            ROS_DEBUG("right arm Inverse kinematics failed at pose no. %d",i);
//            return 0;
//          }
//        }
//        else
//        {
//          ROS_ERROR("right arm Inverse kinematics service call failed at pose no. %d",i);
//          return 0;
//        }
//    }
//    return 1;
//}

///*! \brief Calls IK service.
// *
// */
//bool _call_left_arm_gpik(std::vector<double> &left_joint_trajectory)
//{
//    kinematics_msgs::GetKinematicSolverInfo::Request request;
//    kinematics_msgs::GetKinematicSolverInfo::Response response;
//    std::vector<double> last_joints(7);
//    geometry_msgs::Pose pose;
//    tf::Transform tf_base_wrist, tf_base_obj;

//    if(_query_client_l.call(request,response))
//    {
//      for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
//      {
//        ROS_DEBUG("left_arm Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
//      }
//    }
//    else
//    {
//      ROS_ERROR("Could not call left_arm query service");
//      ros::shutdown();
//      exit(-1);
//    }

//    kinematics_msgs::GetConstraintAwarePositionIK::Request  gpik_req;
//    kinematics_msgs::GetConstraintAwarePositionIK::Response gpik_res;
//    gpik_req.timeout = ros::Duration(5.0);
//    gpik_req.ik_request.ik_link_name = "l_wrist_roll_link";

//    gpik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
//    gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
//    gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;

//    _get_left_joints(last_joints);

//    for(unsigned int i=0; i<_obj_pose_traj.poses.size(); i++)
//    {
//        tf_base_obj = pose2tf(_obj_pose_traj.poses[i]);
//        tf_base_wrist = tf_base_obj*_left_wrist_offset;
//        pose = tf2pose(tf_base_wrist);
//        for(int k=0; k<7; k++)
//            gpik_req.ik_request.ik_seed_state.joint_state.position[k] = last_joints[k];
//        gpik_req.ik_request.pose_stamped.pose = pose;
//        if(ik_client_l_.call(gpik_req, gpik_res))
//        {
//          if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
//          {
//            for(unsigned int j=0; j<gpik_res.solution.joint_state.name.size(); j++)
//            {
//              //ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
//              left_joint_trajectory.push_back(gpik_res.solution.joint_state.position[j]);
//            }
//            for( int l=0; l<7; l++)
//                last_joints[l] = gpik_res.solution.joint_state.position[l];
//          }
//          else
//          {
//            ROS_DEBUG("left_arm Inverse kinematics failed at pose no. %d",i);
//            return 0;
//          }
//        }
//        else
//        {
//          ROS_ERROR("left_arm Inverse kinematics service call failed at pose no. %d",i);
//          return 0;
//        }
//    }
//    return 1;
//}

bool TubeManipulation::Arms::getRightArmIK(geometry_msgs::Pose pose,
                             sensor_msgs::JointState &jointState)
{
    std::vector<double> joints(7);
    _get_right_joints(joints);
    return(_get_right_arm_ik(pose, jointState, joints));
}

bool TubeManipulation::Arms::getSimpleRightArmIK(geometry_msgs::Pose pose,
                             sensor_msgs::JointState &jointState)
{
    std::vector<double> joints(7);
    _get_right_joints(joints);
    return(_get_simple_right_arm_ik(pose, jointState, joints));
}

bool TubeManipulation::Arms::getSimpleRightArmIK(geometry_msgs::Pose pose,
                             std::vector<double> &jointState)
{
    std::vector<double> joints(7);
    _get_right_joints(joints);
    return(_get_simple_right_arm_ik(pose, jointState, joints));
}


bool TubeManipulation::Arms::getLeftArmIK(geometry_msgs::Pose pose,
                             sensor_msgs::JointState &jointState)
{
    std::vector<double> joints(7);
    _get_left_joints(joints);
    return(_get_left_arm_ik(pose, jointState, joints));
}

bool TubeManipulation::Arms::getSimpleLeftArmIK(geometry_msgs::Pose pose,
                             sensor_msgs::JointState &jointState)
{
    std::vector<double> joints(7);
    _get_left_joints(joints);
    return(_get_simple_left_arm_ik(pose, jointState, joints));
}

bool TubeManipulation::Arms::_get_right_arm_ik(geometry_msgs::Pose pose,
                                 sensor_msgs::JointState &joint_state,
                                 std::vector<double> &seed_state)
{
    kinematics_msgs::GetConstraintAwarePositionIK::Request  ik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;

    kinematics_msgs::GetKinematicSolverInfo::Request request;
    kinematics_msgs::GetKinematicSolverInfo::Response response;
    if(_query_client_r.call(request,response))
    {
      for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
      {
        ROS_DEBUG("right_arm Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
      }
    }
    else
    {
      ROS_ERROR("Could not call right_arm query service");
      return false;
    }

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
    if(seed_state.size()==7)
    {
        for(unsigned int i=0; i<7; i++)
            ik_req.ik_request.ik_seed_state.joint_state.position[i] = seed_state[i];
    }
    else
    {
        ROS_ERROR("Seed state value is less than number of joints (7)");
        return 0;
    }

    ik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
    ik_req.ik_request.pose_stamped.pose = pose;

    if(_ik_client_r.call(ik_req, ik_res))
    {
      if(ik_res.error_code.val == ik_res.error_code.SUCCESS)
      {
          joint_state = ik_res.solution.joint_state;
      }
      else
      {
        ROS_WARN("Right arm Inverse kinematics failed for given pose.");
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

bool TubeManipulation::Arms::_get_simple_right_arm_ik(geometry_msgs::Pose &pose,
                                 sensor_msgs::JointState &joint_state,
                                 std::vector<double> &seed_state)
{
    kinematics_msgs::GetPositionIK::Request  ik_req;
    kinematics_msgs::GetPositionIK::Response ik_res;
    kinematics_msgs::GetKinematicSolverInfo::Request request;
    kinematics_msgs::GetKinematicSolverInfo::Response response;

    if(_query_client_r.call(request,response))
    {
      for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
      {
        ROS_DEBUG("right_arm Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
      }
    }
    else
    {
      ROS_ERROR("Could not call right_arm query service");
      return false;
    }

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
    if(seed_state.size()==7)
    {
        for(unsigned int i=0; i<7; i++)
            ik_req.ik_request.ik_seed_state.joint_state.position[i] = seed_state[i];
    }
    else
    {
        ROS_ERROR("Seed state value is less than number of joints (7)");
        return 0;
    }

    ik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
    ik_req.ik_request.pose_stamped.pose = pose;

    if(_smpl_ik_client_r.call(ik_req, ik_res))
    {
      if(ik_res.error_code.val == ik_res.error_code.SUCCESS)
      {
          joint_state = ik_res.solution.joint_state;
      }
      else
      {
        ROS_WARN("Right arm Inverse kinematics failed for given pose.");
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

bool TubeManipulation::Arms::_get_left_arm_ik(geometry_msgs::Pose pose,
                                sensor_msgs::JointState &joint_state,
                                std::vector<double> &seed_state)
{
    kinematics_msgs::GetConstraintAwarePositionIK::Request  ik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;

    kinematics_msgs::GetKinematicSolverInfo::Request request;
    kinematics_msgs::GetKinematicSolverInfo::Response response;
    if(_query_client_l.call(request,response))
    {
      for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
      {
        ROS_DEBUG("left_arm Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
      }
    }
    else
    {
      ROS_ERROR("Could not call left_arm query service");
      return false;
    }

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
    if(seed_state.size()==7)
    {
        for(unsigned int i=0; i<7; i++)
            ik_req.ik_request.ik_seed_state.joint_state.position[i] = seed_state[i];
    }
    else
    {
        ROS_ERROR("Seed state value is less than number of joints (7)");
        return 0;
    }

    ik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
    ik_req.ik_request.pose_stamped.pose = pose;

    if(_ik_client_l .call(ik_req, ik_res))
    {
      if(ik_res.error_code.val == ik_res.error_code.SUCCESS)
      {
          joint_state = ik_res.solution.joint_state;
      }
      else
      {
        ROS_WARN("Left arm Inverse kinematics failed for given pose.");
        return false;
      }
    }
    else
    {
      ROS_ERROR("Left arm Inverse kinematics service call failed.");
      return false;
    }
    return true;
}

bool TubeManipulation::Arms::_get_simple_left_arm_ik(geometry_msgs::Pose &pose,
                                sensor_msgs::JointState &joint_state,
                                std::vector<double> &seed_state)
{
    kinematics_msgs::GetPositionIK::Request  ik_req;
    kinematics_msgs::GetPositionIK::Response ik_res;

    kinematics_msgs::GetKinematicSolverInfo::Request request;
    kinematics_msgs::GetKinematicSolverInfo::Response response;
    if(_query_client_l.call(request,response))
    {
      for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
      {
        ROS_DEBUG("left_arm Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
      }
    }
    else
    {
      ROS_ERROR("Could not call left_arm query service");
      return false;
    }

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
    if(seed_state.size()==7)
    {
        for(unsigned int i=0; i<7; i++)
            ik_req.ik_request.ik_seed_state.joint_state.position[i] = seed_state[i];
    }
    else
    {
        ROS_ERROR("Seed state value is less than number of joints (7)");
        return 0;
    }

    ik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
    ik_req.ik_request.pose_stamped.pose = pose;

    if(_smpl_ik_client_l .call(ik_req, ik_res))
    {
      if(ik_res.error_code.val == ik_res.error_code.SUCCESS)
      {
          joint_state = ik_res.solution.joint_state;
      }
      else
      {
        ROS_WARN("Left arm Inverse kinematics failed for given pose.");
        return false;
      }
    }
    else
    {
      ROS_ERROR("Left arm Inverse kinematics service call failed.");
      return false;
    }
    return true;
}

bool TubeManipulation::Arms::_get_simple_right_arm_ik(geometry_msgs::Pose &pose, std::vector<double> &joints, std::vector<double> &seed_state)
{
    sensor_msgs::JointState joint_state;
    _get_simple_right_arm_ik(pose, joint_state, seed_state);
    joints.resize(joint_state.position.size());
    for(int i=0; i<joint_state.position.size(); i++)
        joints[i] = joint_state.position[i];
}

bool TubeManipulation::Arms::_get_simple_left_arm_ik(geometry_msgs::Pose &pose, std::vector<double> &joints, std::vector<double> &seed_state)
{
    sensor_msgs::JointState joint_state;
    _get_simple_left_arm_ik(pose, joint_state, seed_state);
    joints.resize(joint_state.position.size());
    for(int i=0; i<joint_state.position.size(); i++)
        joints[i] = joint_state.position[i];
}

