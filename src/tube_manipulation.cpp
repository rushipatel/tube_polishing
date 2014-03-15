#include "tubeManipulation.h"


/*! \brief Constructor. Subscribes to various services.
 *
 */
TubeManipulation::Arms::Arms(ros::NodeHandlePtr rh)
{
    _rh = rh;
    _traj_client_r  = new TrajClient("r_arm_controller/joint_trajectory_action", true);
    _traj_client_l  = new TrajClient("l_arm_controller/joint_trajectory_action", true);
    while(!_traj_client_r->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the right_arm_controller/joint_trajectory_action server");
    while(!_traj_client_l->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the left_arm_controller/joint_trajectory_action server");
    _mv_arm_client_r = new MoveArmClient("move_right_arm",true);
    _mv_arm_client_l = new MoveArmClient("move_left_arm",true);
    while(!_mv_arm_client_r->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the move_right_arm action server");
    while(!_mv_arm_client_l->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the move_left_arm action server");
    ros::service::waitForService("pr2_right_arm_kinematics/get_constraint_aware_ik");
    ros::service::waitForService("pr2_right_arm_kinematics/get_ik");
    ros::service::waitForService("pr2_left_arm_kinematics/get_constraint_aware_ik");
    ros::service::waitForService("pr2_left_arm_kinematics/get_ik");
    ros::service::waitForService("trajectory_filter_unnormalizer/filter_trajectory");
    ros::service::waitForService("trajectory_filter_server/filter_trajectory_with_constraints");

    _set_pln_scn_client = _rh->serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
    _fk_client_r = _rh->serviceClient<kinematics_msgs::GetPositionFK>("pr2_right_arm_kinematics/get_fk");
    _fk_client_l = _rh->serviceClient<kinematics_msgs::GetPositionFK>("pr2_left_arm_kinematics/get_fk");
    _ik_client_r = _rh->serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("pr2_right_arm_kinematics/get_constraint_aware_ik");
    _smpl_ik_client_r = _rh->serviceClient<kinematics_msgs::GetPositionIK>("pr2_right_arm_kinematics/get_ik");
        _ik_client_l = _rh->serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("pr2_left_arm_kinematics/get_constraint_aware_ik");
    _smpl_ik_client_l = _rh->serviceClient<kinematics_msgs::GetPositionIK>("pr2_left_arm_kinematics/get_ik");
    _traj_unnormalizer_client =
            _rh->serviceClient<arm_navigation_msgs::FilterJointTrajectory>("trajectory_filter_unnormalizer/filter_trajectory");
    _traj_filter_client = _rh->serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>("trajectory_filter_server/filter_trajectory_with_constraints");
    _att_obj_pub = _rh->advertise<arm_navigation_msgs::AttachedCollisionObject>("attached_collision_object",10);

    arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
    if(!_set_pln_scn_client.call(planning_scene_req, planning_scene_res))
        ROS_ERROR("Can't set planning scene");

    _r_jnt_nms.push_back("r_shoulder_pan_joint");
    _r_jnt_nms.push_back("r_shoulder_lift_joint");
    _r_jnt_nms.push_back("r_upper_arm_roll_joint");
    _r_jnt_nms.push_back("r_elbow_flex_joint");
    _r_jnt_nms.push_back("r_forearm_roll_joint");
    _r_jnt_nms.push_back("r_wrist_flex_joint");
    _r_jnt_nms.push_back("r_wrist_roll_joint");

    _l_jnt_nms.push_back("l_shoulder_pan_joint");
    _l_jnt_nms.push_back("l_shoulder_lift_joint");
    _l_jnt_nms.push_back("l_upper_arm_roll_joint");
    _l_jnt_nms.push_back("l_elbow_flex_joint");
    _l_jnt_nms.push_back("l_forearm_roll_joint");
    _l_jnt_nms.push_back("l_wrist_flex_joint");
    _l_jnt_nms.push_back("l_wrist_roll_joint");

    _collision_check.reset(new CollisionCheck(_rh));

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

geometry_msgs::Pose TubeManipulation::Arms::getRightArmFK()
{
    std::vector<double> joints;
    _get_right_joints(joints);
    return _get_right_fk(joints);
}

geometry_msgs::Pose TubeManipulation::Arms::getRightArmFK(std::vector<double> &right_joints)
{
    return _get_right_fk(right_joints);
}

geometry_msgs::Pose TubeManipulation::Arms::_get_right_fk(std::vector<double> &joints)
{
    kinematics_msgs::GetPositionFK::Request req;
    kinematics_msgs::GetPositionFK::Response res;

    req.fk_link_names.push_back("r_wrist_roll_link");
    req.header.frame_id = "base_link";
    req.header.stamp = ros::Time::now();

    req.robot_state.joint_state.header.frame_id = "base_link";
    req.robot_state.joint_state.header.stamp = ros::Time::now();

    req.robot_state.joint_state.name.resize(joints.size());
    req.robot_state.joint_state.name = _r_jnt_nms;
    req.robot_state.joint_state.position.resize(joints.size());
    for(int i=0; i<joints.size(); i++)
        req.robot_state.joint_state.position[i] = joints[i];

    geometry_msgs::PoseStamped pose_stamped;
    if(_fk_client_r.call(req,res))
        pose_stamped = res.pose_stamped[0];
    else
        ROS_ERROR("Right FK service call failed");
    return pose_stamped.pose;
}

geometry_msgs::Pose TubeManipulation::Arms::getLeftArmFK()
{
    std::vector<double> joints;
    _get_left_joints(joints);
    return _get_left_fk(joints);
}

geometry_msgs::Pose TubeManipulation::Arms::getLeftArmFK(std::vector<double> &left_joints)
{
    return _get_left_fk(left_joints);
}

geometry_msgs::Pose TubeManipulation::Arms::_get_left_fk(std::vector<double> &joints)
{
    kinematics_msgs::GetPositionFK::Request req;
    kinematics_msgs::GetPositionFK::Response res;

    req.fk_link_names.push_back("l_wrist_roll_link");
    req.header.frame_id = "base_link";
    req.header.stamp = ros::Time::now();

    req.robot_state.joint_state.header.frame_id = "base_link";
    req.robot_state.joint_state.header.stamp = ros::Time::now();

    req.robot_state.joint_state.name.resize(joints.size());
    req.robot_state.joint_state.name = _l_jnt_nms;
    req.robot_state.joint_state.position.resize(joints.size());
    for(int i=0; i<joints.size(); i++)
        req.robot_state.joint_state.position[i] = joints[i];

    geometry_msgs::PoseStamped pose_stamped;
    if(_fk_client_l.call(req,res))
        pose_stamped = res.pose_stamped[0];
    else
        ROS_ERROR("Left FK service call failed");
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
bool TubeManipulation::Arms::genTrajectory(std::vector<double> &rightJointTraj, std::vector<double> &leftJointTraj)
{
    rightJointTraj.clear();
    leftJointTraj.clear();
    if(!_gen_trarajectory(rightJointTraj, leftJointTraj))
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

    req.start_state.joint_state.name = _r_jnt_nms;
    req.start_state.joint_state.position.resize(_r_jnt_nms.size());

    for(unsigned int i=0; i<_r_jnt_nms.size(); i++)
        req.start_state.joint_state.position[i] = _right_goal.trajectory.points[1].positions[i];


    req.trajectory.joint_names = _r_jnt_nms;
    req.trajectory.points.resize(_right_goal.trajectory.points.size());

    for(unsigned int i=0; i<_right_goal.trajectory.points.size(); i++)
        req.trajectory.points[i].positions.resize(7);

    for(unsigned int i=0; i<_right_goal.trajectory.points.size(); i++)
    {
        for(int j=0; j<_r_jnt_nms.size(); j++)
            req.trajectory.points[i].positions[j] = _right_goal.trajectory.points[i].positions[j];
    }

    if(_traj_unnormalizer_client.call(req,res))
    {
        if(res.error_code.val == res.error_code.SUCCESS)
        {
            _right_goal.trajectory = res.trajectory;
        }
        else
        {
            ROS_ERROR("Requested right trajectory was not filtered. Error code: %d",res.error_code.val);
            return;
        }
    }
    else
    {
        ROS_ERROR("Service call to right filter trajectory failed %s",_traj_unnormalizer_client.getService().c_str());
        return;
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

    req.start_state.joint_state.name = _l_jnt_nms;
    req.start_state.joint_state.position.resize(_l_jnt_nms.size());

    for(unsigned int i=0; i<7; i++)
        req.start_state.joint_state.position[i] = _left_goal.trajectory.points[1].positions[i];


    req.trajectory.joint_names = _l_jnt_nms;
    req.trajectory.points.resize(_left_goal.trajectory.points.size());

    for(unsigned int i=0; i<_left_goal.trajectory.points.size(); i++)
        req.trajectory.points[i].positions.resize(7);

    for(unsigned int i=0; i<_left_goal.trajectory.points.size(); i++)
    {
        for(int j=0; j<7; j++)
            req.trajectory.points[i].positions[j] = _left_goal.trajectory.points[i].positions[j];
    }

    if(_traj_unnormalizer_client.call(req,res))
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
        ROS_ERROR("Service call to left filter trajectory failed %s",_traj_unnormalizer_client.getService().c_str());
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

    _right_goal.trajectory.joint_names = _r_jnt_nms;
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

    _left_goal.trajectory.joint_names = _l_jnt_nms;
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

// pass empty joint trajectory if that arm is not being used as of now
void TubeManipulation::Arms::_execute_joint_trajectory(trajectory_msgs::JointTrajectory &right_traj, trajectory_msgs::JointTrajectory &left_traj)
{
    pr2_controllers_msgs::JointTrajectoryGoal right_goal, left_goal;

    bool right_arm = false, left_arm =false;
    if(!right_traj.points.empty())
    {
        right_arm = true;
        right_goal.trajectory = right_traj;
    }
    if(!left_traj.points.empty())
    {
        left_arm = true;
        left_goal.trajectory = left_traj;
    }

    ros::Time time_to_start = ros::Time::now()+ros::Duration(3);
    if(right_arm)
    {
        right_goal.trajectory.header.stamp = time_to_start;
        _traj_client_r->sendGoal(right_goal);
    }
    if(left_arm)
    {
        left_goal.trajectory.header.stamp = time_to_start;
        _traj_client_l->sendGoal(left_goal);
    }
    if(right_arm)
        _traj_client_r->waitForResult(ros::Duration(60));
    if(left_arm)
        _traj_client_l->waitForResult(ros::Duration(60));
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
        traj_goal.trajectory.joint_names = _r_jnt_nms;
        goal.positions.resize(_r_jnt_nms.size());
        goal.velocities.resize(_r_jnt_nms.size());
        for(int i=0; i<_r_jnt_nms.size(); i++)
        {
            goal.positions[i] = joint_state.position[i];
            goal.velocities[i] = 0.0;
        }
        goal.time_from_start = ros::Duration(0.0);
        traj_goal.trajectory.points[0] = goal;
        ros::Time time_to_start = ros::Time::now()+ros::Duration(0.1);
        traj_goal.trajectory.header.stamp = time_to_start;
        _traj_client_r->sendGoalAndWait(traj_goal);
        std::vector<double> joints(_r_jnt_nms.size());
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

bool TubeManipulation::Arms::moveRightArmWithMPlanning(arm_navigation_msgs::AttachedCollisionObject &attObj, geometry_msgs::Pose pose)
{
    return _move_right_arm_with_mplning(attObj, pose);
}

bool TubeManipulation::Arms::moveRightArmWithMPlanning(geometry_msgs::Pose pose)
{
    arm_navigation_msgs::AttachedCollisionObject attObj;
    return _move_right_arm_with_mplning(attObj, pose);
}

//bool TubeManipulation::Arms::_move_right_arm_with_mplning(arm_navigation_msgs::AttachedCollisionObject &attObj, geometry_msgs::Pose pose)
//{
//    arm_navigation_msgs::MoveArmGoal goalA;
//    arm_navigation_msgs::SetPlanningSceneDiff::Request req;
//    arm_navigation_msgs::SetPlanningSceneDiff::Response res;
//    if(!attObj.object.shapes.empty())
//        req.planning_scene_diff.attached_collision_objects.push_back(attObj);
//    if(!_set_pln_scn_client.call(req,res))
//        ROS_ERROR("TubeManipulation - Move right arm with MPlanning - Can't set planning scene");

//    std::vector<double> ik_joints(7), joints;
//    _get_right_joints(joints);
//    _get_right_arm_ik(pose,ik_joints,joints);

//    goalA.motion_plan_request.group_name = "right_arm";
//    goalA.motion_plan_request.num_planning_attempts = 10;
//    goalA.motion_plan_request.planner_id = std::string("");
//    goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
//    goalA.motion_plan_request.allowed_planning_time = ros::Duration(60.0);
//    goalA.motion_plan_request.start_state.joint_state.name = _r_jnt_nms;
//    goalA.motion_plan_request.start_state.joint_state.position = joints;
//    goalA.motion_plan_request.goal_constraints.joint_constraints.resize(_r_jnt_nms.size());

//    for (unsigned int i = 0 ; i < goalA.motion_plan_request.goal_constraints.joint_constraints.size(); i++)
//    {
//      goalA.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = _r_jnt_nms[i];
//      goalA.motion_plan_request.goal_constraints.joint_constraints[i].position = ik_joints[i];
//      goalA.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.05;
//      goalA.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.05;
//    }

//    if (_rh->ok())
//    {
//        bool finished_within_time = false;
//        _mv_arm_client_r->sendGoal(goalA);
//        finished_within_time = _mv_arm_client_r->waitForResult(ros::Duration(60.0));
//        if (!finished_within_time)
//        {
//            _mv_arm_client_r->cancelGoal();
//            ROS_INFO("TubeManipulation - Timed out achieving goal");
//            return false;
//        }
//        else
//        {
//            actionlib::SimpleClientGoalState state = _mv_arm_client_r->getState();
//            bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
//            if(success)
//                ROS_INFO("TubeManipulation - Action finished: %s",state.toString().c_str());
//            /*else if(state==_mv_)
//            {
//                _move_right_arm_with_mplning(attObj, pose);
//            }*/
//            else
//            {
//                ROS_INFO("TubeManipulation - Action failed: %s",state.toString().c_str());
//                return false;
//            }
//        }
//    }
//    return true;
//}

bool TubeManipulation::Arms::_move_right_arm_with_mplning(arm_navigation_msgs::AttachedCollisionObject &attObj, geometry_msgs::Pose pose)
{
    arm_navigation_msgs::SetPlanningSceneDiff::Request scn_req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response scn_res;
    if(!attObj.object.shapes.empty())
    {
        scn_req.planning_scene_diff.attached_collision_objects.push_back(attObj);
        ROS_DEBUG("Adding attached object in to planning scene.");
    }
    if(!_set_pln_scn_client.call(scn_req,scn_res))
        ROS_ERROR("TubeManipulation - Move right arm with MPlanning - Can't set planning scene");

    arm_navigation_msgs::GetMotionPlan::Request req;
    arm_navigation_msgs::GetMotionPlan::Response res;

    std::vector<double> ik_joints(7), crnt_joints, crnt_left_joints;
    _get_right_joints(crnt_joints);
    _get_left_joints(crnt_left_joints);
    _get_right_arm_ik(pose,ik_joints,crnt_joints);

    _collision_check->setAttachedObj(attObj);
    _collision_check->enableVisualization();
    _collision_check->setMarkerLifeTime(30);

    /*if(!_collision_check->isStateValid(crnt_joints,crnt_left_joints))
    {
        ROS_WARN("TubeManipulation - Start state is not valid");
        return false;
    }
    ROS_DEBUG("Start state is valid");*/

    if(!_collision_check->isStateValid(ik_joints,crnt_left_joints))
    {
        ROS_WARN("TubeManipulation - Goal state joints found by IK is not valid");
        return false;
    }
    ROS_DEBUG("Goal state is valid");

    req.motion_plan_request.group_name = "right_arm";
    req.motion_plan_request.num_planning_attempts = 3;
    req.motion_plan_request.planner_id = std::string("");
    req.motion_plan_request.allowed_planning_time = ros::Duration(60.0);
    req.motion_plan_request.start_state.joint_state.name = _r_jnt_nms;
    req.motion_plan_request.start_state.joint_state.position = crnt_joints;
    req.motion_plan_request.goal_constraints.joint_constraints.resize(_r_jnt_nms.size());


    for (unsigned int i = 0 ; i < req.motion_plan_request.goal_constraints.joint_constraints.size(); i++)
    {
      req.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = _r_jnt_nms[i];
      req.motion_plan_request.goal_constraints.joint_constraints[i].position = ik_joints[i];
      req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.2;
      req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.2;
    }

    if(!_get_motion_plan(req,res))
        return false;

    trajectory_msgs::JointTrajectory mplan_traj, filtered_traj, empty_traj;
    mplan_traj = res.trajectory.joint_trajectory;
    if(!_filter_trajectory(mplan_traj, filtered_traj, req))
        return false;
    ROS_DEBUG("Trajectory is filtered");
    _execute_joint_trajectory(filtered_traj,empty_traj);
    ROS_DEBUG("Executing joint trajectory");
    return true;
}

bool TubeManipulation::Arms::moveLeftArmWithMPlanning(arm_navigation_msgs::AttachedCollisionObject &attObj, geometry_msgs::Pose pose)
{
    return _move_left_arm_with_mplning(attObj, pose);
}

bool TubeManipulation::Arms::moveLeftArmWithMPlanning(geometry_msgs::Pose pose)
{
    arm_navigation_msgs::AttachedCollisionObject attObj;
    return _move_left_arm_with_mplning(attObj, pose);
}

bool TubeManipulation::Arms::_move_left_arm_with_mplning(arm_navigation_msgs::AttachedCollisionObject &attObj, geometry_msgs::Pose pose)
{
    arm_navigation_msgs::SetPlanningSceneDiff::Request scn_req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response scn_res;
    if(!attObj.object.shapes.empty())
    {
        scn_req.planning_scene_diff.attached_collision_objects.push_back(attObj);
        ROS_DEBUG("Adding attached object in to planning scene.");
    }
    if(!_set_pln_scn_client.call(scn_req,scn_res))
        ROS_ERROR("TubeManipulation - Move left arm with MPlanning - Can't set planning scene");

    arm_navigation_msgs::GetMotionPlan::Request req;
    arm_navigation_msgs::GetMotionPlan::Response res;

    std::vector<double> ik_joints(7), crnt_joints, crnt_right_joints;
    _get_left_joints(crnt_joints);
    _get_right_joints(crnt_right_joints);
    _get_right_arm_ik(pose,ik_joints,crnt_joints);

    _collision_check->setAttachedObj(attObj);
    _collision_check->enableVisualization();
    _collision_check->setMarkerLifeTime(30);

    /*if(!_collision_check->isStateValid(crnt_right_joints,crnt_joints))
    {
        ROS_WARN("TubeManipulation - Start state is not valid");
        return false;
    }
    ROS_DEBUG("Start state is valid");*/

    if(!_collision_check->isStateValid(crnt_right_joints, ik_joints))
    {
        ROS_WARN("TubeManipulation - Goal state joints found by IK is not valid");
        return false;
    }
    ROS_DEBUG("Goal state is valid");

    req.motion_plan_request.group_name = "left_arm";
    req.motion_plan_request.num_planning_attempts = 3;
    req.motion_plan_request.planner_id = std::string("");
    req.motion_plan_request.allowed_planning_time = ros::Duration(60.0);
    req.motion_plan_request.start_state.joint_state.name = _l_jnt_nms;
    req.motion_plan_request.start_state.joint_state.position = crnt_joints;
    req.motion_plan_request.goal_constraints.joint_constraints.resize(_l_jnt_nms.size());

    for (unsigned int i = 0 ; i < req.motion_plan_request.goal_constraints.joint_constraints.size(); i++)
    {
      req.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = _l_jnt_nms[i];
      req.motion_plan_request.goal_constraints.joint_constraints[i].position = ik_joints[i];
      req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.2;
      req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.2;
    }

    if(!_get_motion_plan(req,res))
        return false;

    trajectory_msgs::JointTrajectory mplan_traj, filtered_traj, empty_traj;
    mplan_traj = res.trajectory.joint_trajectory;
    if(!_filter_trajectory(mplan_traj, filtered_traj, req))
        return false;
    ROS_DEBUG("Trajectory is filtered");
    _execute_joint_trajectory(empty_traj, filtered_traj);
    ROS_DEBUG("Executing joint trajectory");
    return true;
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
        traj_goal.trajectory.joint_names = _l_jnt_nms;
        goal.positions.resize(_l_jnt_nms.size());
        goal.velocities.resize(_l_jnt_nms.size());
        for(int i=0; i<_l_jnt_nms.size(); i++)
        {
            goal.positions[i] = joint_state.position[i];
            goal.velocities[i] = 0.0;
        }
        goal.time_from_start = ros::Duration(0.0);
        traj_goal.trajectory.points[0] = goal;
        ros::Time time_to_start = ros::Time::now()+ros::Duration(0.1);
        traj_goal.trajectory.header.stamp = time_to_start;
        _traj_client_l->sendGoalAndWait(traj_goal);
        
        std::vector<double> joints(_l_jnt_nms.size());
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
        traj_goal.trajectory.joint_names = _r_jnt_nms;
        goal.positions.resize(_r_jnt_nms.size());
        goal.velocities.resize(_r_jnt_nms.size());
        for(int j=0; j<_r_jnt_nms.size(); j++)
        {
            goal.positions[j] = joint_state.position[j];
            goal.velocities[j] = 0.0;
        }
        goal.time_from_start = ros::Duration(0.0);
        traj_goal.trajectory.points[0] = goal;
        ros::Time time_to_start = ros::Time::now()+ros::Duration(0.1);
        traj_goal.trajectory.header.stamp = time_to_start;
        _traj_client_r->sendGoalAndWait(traj_goal);
        std::vector<double> joints(_r_jnt_nms.size());
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
        traj_goal.trajectory.joint_names = _l_jnt_nms;
        goal.positions.resize(_l_jnt_nms.size());
        goal.velocities.resize(_l_jnt_nms.size());
        for(int j=0; j<_l_jnt_nms.size(); j++)
        {
            goal.positions[j] = joint_state.position[j];
            goal.velocities[j] = 0.0;
        }
        goal.time_from_start = ros::Duration(0.0);
        traj_goal.trajectory.points[0] = goal;
        ros::Time time_to_start = ros::Time::now()+ros::Duration(0.1);
        traj_goal.trajectory.header.stamp = time_to_start;
        _traj_client_l->sendGoalAndWait(traj_goal);
        std::vector<double> joints(_l_jnt_nms.size());
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

bool TubeManipulation::Arms::getRightArmIK(geometry_msgs::Pose pose,
                             sensor_msgs::JointState &jointState)
{
    std::vector<double> joints(7);
    _get_right_joints(joints);
    return(_get_right_arm_ik(pose, jointState, joints));
}

bool TubeManipulation::Arms::getRightArmIK(geometry_msgs::Pose pose,
                             std::vector<double> &jointsOut)
{
    std::vector<double> joints(7);
    _get_right_joints(joints);
    return(_get_right_arm_ik(pose, jointsOut, joints));
}

bool TubeManipulation::Arms::getSimpleRightArmIK(geometry_msgs::Pose pose,
                             sensor_msgs::JointState &jointState)
{
    std::vector<double> joints(7);
    _get_right_joints(joints);
    return(_get_simple_right_arm_ik(pose, jointState, joints));
}

bool TubeManipulation::Arms::getSimpleRightArmIK(geometry_msgs::Pose pose,
                             std::vector<double> &jointsOut)
{
    std::vector<double> joints(7);
    _get_right_joints(joints);
    return(_get_simple_right_arm_ik(pose, jointsOut, joints));
}


bool TubeManipulation::Arms::getLeftArmIK(geometry_msgs::Pose pose,
                             sensor_msgs::JointState &jointState)
{
    std::vector<double> joints(7);
    _get_left_joints(joints);
    return(_get_left_arm_ik(pose, jointState, joints));
}

bool TubeManipulation::Arms::getLeftArmIK(geometry_msgs::Pose pose,
                             std::vector<double> &jointsOut)
{
    std::vector<double> joints(7);
    _get_left_joints(joints);
    return(_get_left_arm_ik(pose, jointsOut, joints));
}

bool TubeManipulation::Arms::getSimpleLeftArmIK(geometry_msgs::Pose pose,
                             sensor_msgs::JointState &jointState)
{
    std::vector<double> joints(7);
    _get_left_joints(joints);
    return(_get_simple_left_arm_ik(pose, jointState, joints));
}

bool TubeManipulation::Arms::_get_right_arm_ik(geometry_msgs::Pose &pose, std::vector<double> &joints, std::vector<double> &seed_state)
{
    sensor_msgs::JointState joint_state;
    bool flag = _get_right_arm_ik(pose, joint_state, seed_state);
    joints.resize(joint_state.position.size());
    for(int i=0; i<joint_state.position.size(); i++)
        joints[i] = joint_state.position[i];
    return flag;
}

bool TubeManipulation::Arms::_get_simple_right_arm_ik(geometry_msgs::Pose &pose, std::vector<double> &joints, std::vector<double> &seed_state)
{
    sensor_msgs::JointState joint_state;
    bool flag = _get_simple_right_arm_ik(pose, joint_state, seed_state);
    joints.resize(joint_state.position.size());
    for(int i=0; i<joint_state.position.size(); i++)
        joints[i] = joint_state.position[i];
    return flag;
}

bool TubeManipulation::Arms::_get_left_arm_ik(geometry_msgs::Pose &pose, std::vector<double> &joints, std::vector<double> &seed_state)
{
    sensor_msgs::JointState joint_state;
    bool flag = _get_left_arm_ik(pose, joint_state, seed_state);
    joints.resize(joint_state.position.size());
    for(int i=0; i<joint_state.position.size(); i++)
        joints[i] = joint_state.position[i];
    return flag;
}

bool TubeManipulation::Arms::_get_simple_left_arm_ik(geometry_msgs::Pose &pose, std::vector<double> &joints, std::vector<double> &seed_state)
{
    sensor_msgs::JointState joint_state;
    bool flag = _get_simple_left_arm_ik(pose, joint_state, seed_state);
    joints.resize(joint_state.position.size());
    for(int i=0; i<joint_state.position.size(); i++)
        joints[i] = joint_state.position[i];
    return flag;
}

bool TubeManipulation::Arms::_get_right_arm_ik(geometry_msgs::Pose pose,
                                 sensor_msgs::JointState &joint_state,
                                 std::vector<double> &seed_state)
{
    kinematics_msgs::GetConstraintAwarePositionIK::Request  ik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;

    ik_req.timeout = ros::Duration(5.0);
    ik_req.ik_request.ik_link_name = "r_wrist_roll_link";
    ik_req.ik_request.ik_seed_state.joint_state.name = _r_jnt_nms;
    ik_req.ik_request.ik_seed_state.joint_state.position.resize(_r_jnt_nms.size());
    if(seed_state.size()==_r_jnt_nms.size())
    {
        for(unsigned int i=0; i<_r_jnt_nms.size(); i++)
            ik_req.ik_request.ik_seed_state.joint_state.position[i] = seed_state[i];
    }
    else
    {
        ROS_ERROR("Seed state value is less than number of joints (%d)",_r_jnt_nms.size());
        return 0;
    }

    ik_req.ik_request.pose_stamped.header.frame_id = "base_link";
    ik_req.ik_request.pose_stamped.pose = pose;

    if(_ik_client_r.call(ik_req, ik_res))
    {
      if(ik_res.error_code.val == ik_res.error_code.SUCCESS)
      {
          joint_state = ik_res.solution.joint_state;
      }
      else
      {
        ROS_DEBUG("Right arm Inverse kinematics failed for given pose.");
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

    ik_req.timeout = ros::Duration(5.0);
    ik_req.ik_request.ik_link_name = "r_wrist_roll_link";
    ik_req.ik_request.ik_seed_state.joint_state.name = _r_jnt_nms;
    ik_req.ik_request.ik_seed_state.joint_state.position.resize(_r_jnt_nms.size());
    if(seed_state.size()==_r_jnt_nms.size())
    {
        for(unsigned int i=0; i<_r_jnt_nms.size(); i++)
            ik_req.ik_request.ik_seed_state.joint_state.position[i] = seed_state[i];
    }
    else
    {
        ROS_ERROR("Seed state value is less than number of joints (%d)", _r_jnt_nms.size());
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
        ROS_DEBUG("Right arm Inverse kinematics failed for given pose.");
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

    ik_req.timeout = ros::Duration(5.0);
    ik_req.ik_request.ik_link_name = "l_wrist_roll_link";
    ik_req.ik_request.ik_seed_state.joint_state.name = _l_jnt_nms;
    ik_req.ik_request.ik_seed_state.joint_state.position.resize(_l_jnt_nms.size());

    if(seed_state.size()==_l_jnt_nms.size())
    {
        for(unsigned int i=0; i<_l_jnt_nms.size(); i++)
            ik_req.ik_request.ik_seed_state.joint_state.position[i] = seed_state[i];
    }
    else
    {
        ROS_ERROR("Seed state value is less than number of joints (7)");
        return 0;
    }

    ik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
    ik_req.ik_request.pose_stamped.pose = pose;

    if(_ik_client_l.call(ik_req, ik_res))
    {
      if(ik_res.error_code.val == ik_res.error_code.SUCCESS)
      {
          joint_state = ik_res.solution.joint_state;
      }
      else
      {
        ROS_DEBUG("Left arm Inverse kinematics failed for given pose.");
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

    ik_req.timeout = ros::Duration(5.0);
    ik_req.ik_request.ik_link_name = "l_wrist_roll_link";
    ik_req.ik_request.ik_seed_state.joint_state.name = _l_jnt_nms;
    ik_req.ik_request.ik_seed_state.joint_state.position.resize(_l_jnt_nms.size());
    if(seed_state.size()==_l_jnt_nms.size())
    {
        for(unsigned int i=0; i<_l_jnt_nms.size(); i++)
            ik_req.ik_request.ik_seed_state.joint_state.position[i] = seed_state[i];
    }
    else
    {
        ROS_ERROR("Seed state value is less than number of joints (%d)", _l_jnt_nms.size());
        return 0;
    }

    ik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
    ik_req.ik_request.pose_stamped.pose = pose;

    if(_smpl_ik_client_l.call(ik_req, ik_res))
    {
      if(ik_res.error_code.val == ik_res.error_code.SUCCESS)
      {
          joint_state = ik_res.solution.joint_state;
      }
      else
      {
        ROS_DEBUG("Left arm Inverse kinematics failed for given pose.");
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

//for object held up by right arm
bool TubeManipulation::Arms::getRegraspPoseRight(geometry_msgs::Pose crnt_grasp,
                                                     geometry_msgs::Pose wrist_pose,
                                                     geometry_msgs::Pose other_hand_grasp,
                                                     arm_navigation_msgs::AttachedCollisionObject att_obj,
                                                     geometry_msgs::Pose &obj_pose_out)
{
    bool pose_found = false;
    TubeManipulation::CollisionCheck collision_check(_rh);
    collision_check.setAttachedObj(att_obj);
    collision_check.enableVisualization();
    tf::Transform obj_orig, obj, rand_tf,
            wrist=pose2tf(wrist_pose),
            grasp=pose2tf(crnt_grasp),
            og = pose2tf(other_hand_grasp);

    obj_orig = wrist * grasp.inverse();

    tf::Quaternion q;
    tf::Vector3 pos;
    double y,p,r;
    double x_pos, y_pos, z_pos;
    int MAX_CNT = 10000;
    int cnt = MAX_CNT;
    obj = obj_orig;
    std::vector<double> right_joints(7), right_seeds(7),
                        left_seeds(7), left_joints(7);

    _get_right_joints(right_seeds);
    _get_left_joints(left_seeds);
    tf::Transform wrist_crnt, other_wrist;
    geometry_msgs::Pose wrist_crnt_pose, other_wrist_pose;

    /*visualization_msgs::Marker search_box;
    search_box.type = search_box.CUBE;
    search_box.action = search_box.ADD;
    search_box.color.a = 0.2;
    search_box.color.g = 1;
    search_box.ns = "search_box";
    search_box.id = 1;
    search_box.pose.orientation.x = 0;
    search_box.pose.orientation.y = 0;
    search_box.pose.orientation.z = 0;
    search_box.pose.orientation.w = 1;
    search_box.header.frame_id = "base_link";
    search_box.header.stamp = ros::Time::now();
    search_box.scale.x = 1;
    search_box.scale.y = 1;
    search_box.scale.z = 1;
    search_box.lifetime = ros::Duration(60);*/

    do
    {
        wrist_crnt = obj * grasp;
        other_wrist = obj * og;
        wrist_crnt_pose = tf2pose(wrist_crnt);
        other_wrist_pose = tf2pose(other_wrist);
        if(_get_simple_right_arm_ik(wrist_crnt_pose, right_joints, right_seeds)
           &&_get_simple_left_arm_ik(other_wrist_pose, left_joints, left_seeds) )
        {
            if(collision_check.isStateValid(right_joints, left_joints))
            {
                collision_check.setMarkerLifeTime(60);
                collision_check.isStateValid(right_joints, left_joints);
                ROS_INFO("TubeManipulation - Valid object pose found in %d iteration", (MAX_CNT-cnt));
                cnt = 0;
                obj_orig = obj;
                pose_found = true;
            }
        }

        y = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
        p = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
        r = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
        q.setRPY(r,p,y);

        // bounding box to move around is 0.5x0.5x0.5
        double x_min = 0,
                x_max = 0,
                y_min = 0,
                y_max = 0,
                z_min = 0,
                z_max = 0.5;



        // random number between 0 and 1
        x_pos = ((double)rand()/(double)RAND_MAX);
        y_pos = ((double)rand()/(double)RAND_MAX);
        z_pos = ((double)rand()/(double)RAND_MAX);

        double len;
        len = x_max - x_min;
        x_pos = x_min + (len * x_pos);
        len = y_max - y_min;
        y_pos = y_min + (len * y_pos);
        len = z_max - z_min;
        z_pos = z_min + (len * z_pos);

        pos.setValue(x_pos, y_pos, z_pos);
        rand_tf.setRotation(q);
        rand_tf.setOrigin(pos);
        obj = obj_orig * rand_tf;
        cnt--;
        }while(cnt>0);

    obj_pose_out = tf2pose(obj_orig);

    if(!pose_found)
        ROS_WARN("TubeManipulation - Couldn't find valid object pose for regrasping");

    return pose_found;
}

//for object held by left hand
bool TubeManipulation::Arms::getRegraspPoseLeft(geometry_msgs::Pose crnt_grasp,
                                                     geometry_msgs::Pose wrist_pose,
                                                     geometry_msgs::Pose other_hand_grasp,
                                                     arm_navigation_msgs::AttachedCollisionObject att_obj,
                                                     geometry_msgs::Pose &obj_pose_out)
{
    bool pose_found = false;
    TubeManipulation::CollisionCheck collision_check(_rh);
    collision_check.setAttachedObj(att_obj);
    tf::Transform obj_orig, obj, rand_tf,
            wrist=pose2tf(wrist_pose),
            grasp=pose2tf(crnt_grasp),
            og = pose2tf(other_hand_grasp);

    obj_orig = wrist * grasp.inverse();

    tf::Quaternion q;
    tf::Vector3 pos;
    double y,p,r;
    double x_pos, y_pos, z_pos;
    int MAX_CNT = 10000;
    int cnt = MAX_CNT;
    obj = obj_orig;
    std::vector<double> right_joints(7), right_seeds(7),
                        left_seeds(7), left_joints(7);

    _get_right_joints(right_seeds);
    _get_left_joints(left_seeds);
    tf::Transform wrist_crnt, other_wrist;
    geometry_msgs::Pose wrist_crnt_pose, other_wrist_pose;

    do
    {
        wrist_crnt = obj * grasp;
        other_wrist = obj * og;
        wrist_crnt_pose = tf2pose(wrist_crnt);
        other_wrist_pose = tf2pose(other_wrist);
        if(_get_simple_left_arm_ik(wrist_crnt_pose, left_joints, left_seeds)
           &&_get_simple_right_arm_ik(other_wrist_pose, right_joints, right_seeds) )
        {
            if(collision_check.isStateValid(right_joints, left_joints))
            {
                collision_check.setMarkerLifeTime(60);
                collision_check.isStateValid(right_joints, left_joints);
                ROS_INFO("TubeManipulation - Valid object pose found in %d iteration", (MAX_CNT-cnt));
                cnt = 0;
                obj_orig = obj;
                pose_found = true;
            }
        }


        y = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
        p = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
        r = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
        q.setRPY(r,p,y);

        // bounding box to move around is 0.5x0.5x0.5
        double x_min = -0.25,
                x_max = 0.25,
                y_min = -0.5,
                y_max = 0.5,
                z_min = 0,
                z_max = 0.5;

        // random number between 0 and 1
        x_pos = ((double)rand()/(double)RAND_MAX);
        y_pos = ((double)rand()/(double)RAND_MAX);
        z_pos = ((double)rand()/(double)RAND_MAX);

        double len;
        len = x_max - x_min;
        x_pos = x_min + (len * x_pos);
        len = y_max - y_min;
        y_pos = y_min + (len * y_pos);
        len = z_max - z_min;
        z_pos = z_min + (len * z_pos);

        pos.setValue(x_pos, y_pos, z_pos);
        rand_tf.setRotation(q);
        rand_tf.setOrigin(pos);
        obj = obj_orig * rand_tf;
        cnt--;
        }while(cnt>0);

    obj_pose_out = tf2pose(obj_orig);

    if(!pose_found)
        ROS_WARN("TubeManipulation - Couldn't find valid object pose for regrasping");

    return pose_found;
}


bool TubeManipulation::Arms::_get_motion_plan(arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res)
{
    while(!ros::service::waitForService("ompl_planning/plan_kinematic_path",1))
        ROS_INFO("waiting for ompl_planning/plan_kinematic_path");
    ros::ServiceClient srv_client_ = _rh->serviceClient<arm_navigation_msgs::GetMotionPlan>("ompl_planning/plan_kinematic_path");
    if(srv_client_.call(req,res))
    {
        if(res.trajectory.joint_trajectory.points.empty())
        {
            ROS_WARN_STREAM("TubeManipulation - Planner returned with no trajectory. Returned with error code "<<res.error_code.val);
            return false;
        }
        else
            ROS_INFO("TubeManipulation - Motion planning request returned with %d trajectory points.",res.trajectory.joint_trajectory.points.size());
    }
    else
    {
        ROS_ERROR_STREAM("TubeManipulation - call to ompl_planning/plan_kinematic_path service failed. Error code : "<<res.error_code.val);
        return false;
    }
    ROS_DEBUG("Motion plan request returned with solution");
    return true;
}

// from move_arm.cpp ros fuerte

bool TubeManipulation::Arms::_filter_trajectory(trajectory_msgs::JointTrajectory &trajectory_in,
                      trajectory_msgs::JointTrajectory &trajectory_out, arm_navigation_msgs::GetMotionPlan::Request mplan_req)
{
    if(trajectory_in.points.empty())
    {
        ROS_WARN("No trajectory point");
        return false;
    }
    arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request req;
    arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response res;

    req.goal_constraints = mplan_req.motion_plan_request.goal_constraints;
    req.path_constraints = mplan_req.motion_plan_request.path_constraints;

    req.group_name =  mplan_req.motion_plan_request.group_name;

    req.allowed_time = ros::Duration(5);
    req.start_state.joint_state.name = trajectory_in.joint_names;
    req.start_state.joint_state.position.resize(trajectory_in.joint_names.size());

    trajectory_msgs::JointTrajectoryPoint traj_point = trajectory_in.points[0];
    req.start_state.joint_state.position = traj_point.positions;

    req.trajectory = trajectory_in;

    if(_traj_filter_client.call(req,res))
    {
        if(res.error_code.val == res.error_code.SUCCESS)
        {
            trajectory_out = res.trajectory;
        }
        else
        {
            ROS_ERROR("Trajectory was not filtered. Error code: %d",res.error_code.val);
            return false;
        }
    }
    else
    {
        ROS_ERROR("Service call to filter_trajectory failed %s",_traj_filter_client.getService().c_str());
        return false;
    }
    return true;
}

TubeManipulation::CollisionCheck::CollisionCheck(ros::NodeHandlePtr nh)
{
    _nh = nh;
    _mrkr_pub = _nh->advertise<visualization_msgs::MarkerArray>("tube_polishing/state_validity",100);
    _collision_models = new planning_environment::CollisionModels("robot_description");
    ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
    _get_scn_client = _nh->serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);

    _visualize = false;
    _good_color.a = _collision_color.a = _joint_limits_color.a = .8;
    _good_color.g = 1.0;
    _collision_color.r = 1.0;
    _joint_limits_color.b = 1.0;
    _point_markers.a = 1.0;
    _point_markers.r = 1.0;
    _point_markers.g = .8;
    _mrk_life_time = 0.2;


    /*_r_jnts.resize(7);
    _l_jnts.resize(7);
    _r_jnt_nms.resize(7);
    _l_jnt_nms.resize(7);*/

    _reset_state();
}

void TubeManipulation::CollisionCheck::enableVisualization()
{
    _visualize = true;
}
void TubeManipulation::CollisionCheck::disableVisualization()
{
    _visualize = false;
}

TubeManipulation::CollisionCheck::~CollisionCheck()
{
        _collision_models->revertPlanningScene(_state);
}

void TubeManipulation::CollisionCheck::setAttachedObj(arm_navigation_msgs::AttachedCollisionObject &attachedObj)
{
    _att_obj = attachedObj;
    refreshState();
}

void TubeManipulation::CollisionCheck::clearAttachedObj()
{
    arm_navigation_msgs::AttachedCollisionObject obj;
    _att_obj = obj;
    refreshState();
}

void TubeManipulation::CollisionCheck::refreshState(void)
{
    _collision_models->revertPlanningScene(_state);
    _reset_state();
}

void TubeManipulation::CollisionCheck::_reset_state(void)
{

    if(!_att_obj.object.shapes.empty())
        _scn_req.planning_scene_diff.attached_collision_objects.push_back(_att_obj);
    else
        _scn_req.planning_scene_diff.attached_collision_objects.clear();

    if(!_get_scn_client.call(_scn_req,_scn_res))
    {
        ROS_ERROR("Can't get planning scene");
        return;
    }
    _scn = _scn_res.planning_scene;
    _state = _collision_models->setPlanningScene(_scn);
    _r_jnt_nms = _collision_models->getKinematicModel()->getModelGroup("right_arm")->getJointModelNames();
    _l_jnt_nms = _collision_models->getKinematicModel()->getModelGroup("left_arm")->getJointModelNames();
    _r_lnk_nms = _collision_models->getKinematicModel()->getModelGroup("right_arm")->getUpdatedLinkModelNames();
    _l_lnk_nms = _collision_models->getKinematicModel()->getModelGroup("left_arm")->getUpdatedLinkModelNames();

    _state->getJointStateGroup("right_arm")->getKinematicStateValues(_jnt_values);
    _actual_r_jnts.resize(_r_jnt_nms.size());
    for(size_t i=0; i<_r_jnt_nms.size(); i++)
        _actual_r_jnts[i] = _jnt_values[_r_jnt_nms[i].c_str()];

    _actual_l_jnts.resize(_l_jnt_nms.size());
    _state->getJointStateGroup("left_arm")->getKinematicStateValues(_jnt_values);
    for(size_t i=0; i<_l_jnt_nms.size(); i++)
        _actual_l_jnts[i] = _jnt_values[_l_jnt_nms[i].c_str()];
}

void TubeManipulation::CollisionCheck::printState(void)
{
    refreshState();
    std::cout<<std::endl;
    std::cout<<"****** State Output *******"<<std::endl;

    std::cout<<"Right Joint Names:"<<std::endl;
    for(int i=0; i<_r_jnt_nms.size(); i++)
        std::cout<<i<<"  "<<_r_jnt_nms[i]<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Left Joint Names:"<<std::endl;
    for(int i=0; i<_l_jnt_nms.size(); i++)
        std::cout<<i<<"  "<<_l_jnt_nms[i]<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Right Link Names:"<<std::endl;
    for(int i=0; i<_r_lnk_nms.size(); i++)
        std::cout<<i<<"  "<<_r_lnk_nms[i]<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Left Link Names:"<<std::endl;
    for(int i=0; i<_l_lnk_nms.size(); i++)
        std::cout<<i<<"  "<<_l_lnk_nms[i]<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Joint Values:"<<std::endl;
    std::cout<<"    \tRight"<<"\t\t\t"<<"Left"<<std::endl;
    for(int i=0; i<_actual_r_jnts.size(); i++)
    {
        std::cout<<i<<" > \t"<<_actual_r_jnts[i]<<"\t\t"<<_actual_l_jnts[i]<<std::endl;
    }
    std::cout<<std::endl;
}

bool TubeManipulation::CollisionCheck::isStateValid(std::vector<double> &right_joints, std::vector<double> &left_joints)
{
    ROS_ASSERT_MSG(right_joints.size()==7,"TubeManipulation - Number of right joint values is not 7");
    ROS_ASSERT_MSG(left_joints.size()==7,"TubeManipulation - Number of left joint values is not 7");

    _r_jnts = right_joints;
    _l_jnts = left_joints;

    _jnt_values.clear();

    for(size_t i=0; i<_r_jnts.size(); i++)
    {
        _jnt_values[_r_jnt_nms[i]] = _r_jnts[i];
        _jnt_values[_l_jnt_nms[i]] = _l_jnts[i];
    }
    return _is_state_valid();
}

bool TubeManipulation::CollisionCheck::_is_state_valid()
{

    bool is_valid = true;

    std_msgs::ColorRGBA color;
    color = _good_color;


    _state->setKinematicState(_jnt_values);
    if(!_state->areJointsWithinBounds(_r_jnt_nms))
    {
        ROS_DEBUG("TubeManipulation - Right joints are out of bound");
        color = _joint_limits_color;
        is_valid = false;
    }
    if(!_state->areJointsWithinBounds(_l_jnt_nms))
    {
        ROS_DEBUG("TubeManipulation - Left joints are out of bound");
        color = _joint_limits_color;
        is_valid = false;
    } else if(_collision_models->isKinematicStateInCollision(*_state))
    {
        std::stringstream ss;
        ss.flush();
        color = _collision_color;
        is_valid = false;
        if(_collision_models->isKinematicStateInEnvironmentCollision(*_state))
            ss<<"environment ";
        if(_collision_models->isKinematicStateInSelfCollision(*_state))
        {
            if(ss.gcount()>5)
                ss<<"and self";
            else
                ss<<"self";
        }
        ROS_DEBUG_STREAM("TubeManipulation - kinematic state is in collision with "<<ss.str());
        _collision_models->getAllCollisionPointMarkers(*_state, _mrkr_arr, _point_markers, ros::Duration(_mrk_life_time));
    }

    if(_visualize)
    {
        _collision_models->getRobotMarkersGivenState(*_state, _mrkr_arr, color,"right_arm",ros::Duration(_mrk_life_time),&_r_lnk_nms);
        _collision_models->getRobotMarkersGivenState(*_state, _mrkr_arr, color,"left_arm",ros::Duration(_mrk_life_time),&_l_lnk_nms);
        _collision_models->getAttachedCollisionObjectMarkers(*_state, _mrkr_arr, "right_arm",color,ros::Duration(_mrk_life_time), false, &_r_lnk_nms);
        _collision_models->getAttachedCollisionObjectMarkers(*_state, _mrkr_arr, "left_arm",color,ros::Duration(_mrk_life_time), false, &_l_lnk_nms);
        _mrkr_pub.publish(_mrkr_arr);
        _mrkr_arr.markers.clear();
    }
    return is_valid;
}

void TubeManipulation::CollisionCheck::setMarkerLifeTime(double time)
{
    _mrk_life_time = time;
}
