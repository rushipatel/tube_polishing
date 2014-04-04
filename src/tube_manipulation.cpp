#include "tubeManipulation.h"
#include <arm_kinematics_constraint_aware/arm_kinematics_solver_constraint_aware.h>
#include <stdio.h>

#define ARMS_LGRNM "Arms"
#define COLCHK_LGRNM "CollisionCheck"
#define TRAJ_LGRNM "Trajectory"
#define MAX_VELOCITY 0.2

TubeManipulation::Trajectory::Trajectory()
{
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

    ros::Time time = ros::Time::now();
    _right_traj.header.frame_id = "base_link";
    _right_traj.header.stamp = time;
    _right_traj.joint_names = _r_jnt_nms;

    _left_traj.header.frame_id = "base_link";
    _left_traj.header.stamp = time;
    _left_traj.joint_names = _l_jnt_nms;
}

void TubeManipulation::Trajectory::setTrajectory(std::vector<double> &rightTraj, std::vector<double> &leftTraj)
{
    trajectory_msgs::JointTrajectoryPoint point;
    double nr_of_joints = _r_jnt_nms.size();
    point.positions.resize(_r_jnt_nms.size());
    if( (rightTraj.size()%_r_jnt_nms.size()) != 0){
        ROS_ERROR_NAMED(TRAJ_LGRNM,"Couldn't set trajectory from linear vector. size of vector is not in multiple of %d",_r_jnt_nms.size());
        return;
    }
    unsigned int nr_of_points = rightTraj.size()/nr_of_joints;
    _right_traj.points.resize(nr_of_points);
    for(unsigned int i=0; i<_right_traj.points.size(); i++){
        for(unsigned int j=0; j<nr_of_joints; j++)
            point.positions[j] = rightTraj[(i*7) + j];
        _right_traj.points[i] = point;
    }

    nr_of_points = leftTraj.size()/nr_of_joints;
    _left_traj.points.resize(nr_of_points);
    for(unsigned int i=0; i<_left_traj.points.size(); i++){
        for(unsigned int j=0; j<nr_of_joints; j++)
            point.positions[j] = leftTraj[(i*7) + j];
        _left_traj.points[i] = point;
    }
}

/*void TubeManipulation::Trajectory::syncTrajectories()
{
    ;
}*/


/*! \brief Constructor. Subscribes to various services.
 *
 */
TubeManipulation::Arms::Arms(ros::NodeHandlePtr nh, collisionObjects::Ptr colObjPtr)
{
    _rh = nh;
    _collision_objects = colObjPtr;
    _traj_client_r  = new TrajClient("r_arm_controller/joint_trajectory_action", true);
    _traj_client_l  = new TrajClient("l_arm_controller/joint_trajectory_action", true);
    while(!_traj_client_r->waitForServer(ros::Duration(5.0)))
        ROS_INFO_NAMED(ARMS_LGRNM,"Waiting for the right_arm_controller/joint_trajectory_action server");
    while(!_traj_client_l->waitForServer(ros::Duration(5.0)))
        ROS_INFO_NAMED(ARMS_LGRNM,"Waiting for the left_arm_controller/joint_trajectory_action server");

    ros::service::waitForService("pr2_right_arm_kinematics/get_constraint_aware_ik");
    ros::service::waitForService("pr2_right_arm_kinematics/get_ik");
    ros::service::waitForService("pr2_left_arm_kinematics/get_constraint_aware_ik");
    ros::service::waitForService("pr2_left_arm_kinematics/get_ik");
    ros::service::waitForService("trajectory_filter_unnormalizer/filter_trajectory");
    ros::service::waitForService("trajectory_filter_server/filter_trajectory_with_constraints");

    //_set_pln_scn_client = _rh->serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
    _fk_client_r = _rh->serviceClient<kinematics_msgs::GetPositionFK>("pr2_right_arm_kinematics/get_fk");
    _fk_client_l = _rh->serviceClient<kinematics_msgs::GetPositionFK>("pr2_left_arm_kinematics/get_fk");
    _ik_client_r = _rh->serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("pr2_right_arm_kinematics/get_constraint_aware_ik");
    _smpl_ik_client_r = _rh->serviceClient<kinematics_msgs::GetPositionIK>("pr2_right_arm_kinematics/get_ik");
        _ik_client_l = _rh->serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("pr2_left_arm_kinematics/get_constraint_aware_ik");
    _smpl_ik_client_l = _rh->serviceClient<kinematics_msgs::GetPositionIK>("pr2_left_arm_kinematics/get_ik");
    _traj_unnormalizer_client =
            _rh->serviceClient<arm_navigation_msgs::FilterJointTrajectory>("trajectory_filter_unnormalizer/filter_trajectory");
    _traj_filter_client = _rh->serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>("trajectory_filter_server/filter_trajectory_with_constraints");

    _test_pose_pub = _rh->advertise<geometry_msgs::PoseStamped>("tube_polishing/arms/test_pose", 1);

    /*arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
    ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
    if(!_set_pln_scn_client.call(planning_scene_req, planning_scene_res))
        ROS_ERROR_NAMED(ARMS_LGRNM,"Arms - Can't get planning scene");*/

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

    _collision_check.reset(new CollisionCheck(_rh, _collision_objects));
    _collision_check->setMarkerLifeTime(0.25);
    _collision_check->enableVisualization();
    _get_bounds_from_description();
}

TubeManipulation::Arms::~Arms(){
    delete _traj_client_r;
    delete _traj_client_l;
}

void TubeManipulation::Arms::_get_bounds_from_description()
{
    planning_environment::CollisionModels collision_model("robot_description");

    std::pair<double, double> bounds;
    for(int i=0; i<_r_jnt_nms.size(); i++){
        const planning_models::KinematicModel::JointModel* jnt_mdl =
                collision_model.getKinematicModel()->getJointModel(_r_jnt_nms[i].c_str());
        jnt_mdl->getVariableBounds(_r_jnt_nms[i].c_str(), bounds);
        _joint_bounds[_r_jnt_nms[i].c_str()] = bounds;
    }

    for(int i=0; i<_l_jnt_nms.size(); i++){
        const planning_models::KinematicModel::JointModel* jnt_mdl =
                collision_model.getKinematicModel()->getJointModel(_l_jnt_nms[i].c_str());
        jnt_mdl->getVariableBounds(_l_jnt_nms[i].c_str(), bounds);
        _joint_bounds[_l_jnt_nms[i].c_str()] = bounds;
    }
}

void TubeManipulation::Arms::getRightJoints(std::vector<double> &joints){
    _get_right_joints(joints);
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

    joint_state.resize(_r_jnt_nms.size());
    //extract the joint angles from it
    for(unsigned int i=0; i<_r_jnt_nms.size(); i++)
      joint_state[i] = state_msg->actual.positions[i];
}

void TubeManipulation::Arms::getLeftJoints(std::vector<double> &joints){
    _get_left_joints(joints);
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
    joint_state.resize(_l_jnt_nms.size());
    //extract the joint angles from message
    for(unsigned int i=0; i<_l_jnt_nms.size(); i++)
      joint_state[i] = state_msg->actual.positions[i];
}

geometry_msgs::Pose TubeManipulation::Arms::getRightArmFK(){
    std::vector<double> joints;
    _get_right_joints(joints);
    return _get_right_fk(joints);
}

geometry_msgs::Pose TubeManipulation::Arms::getRightArmFK(std::vector<double> &right_joints){
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
        ROS_ERROR_NAMED(ARMS_LGRNM,"Right FK service call failed");
    return pose_stamped.pose;
}

geometry_msgs::Pose TubeManipulation::Arms::getLeftArmFK(){
    std::vector<double> joints;
    _get_left_joints(joints);
    return _get_left_fk(joints);
}

geometry_msgs::Pose TubeManipulation::Arms::getLeftArmFK(std::vector<double> &left_joints){
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
        ROS_ERROR_NAMED(ARMS_LGRNM,"Left FK service call failed");
    return pose_stamped.pose;
}

/*! \brief Generates trajectory by calling IK service and stores output in double linear vector.
 *
 *  Returns false if IK fails at any trajectory point.
 */
bool TubeManipulation::Arms::genTrajectory(geometry_msgs::PoseArray &objPoseArray, geometry_msgs::Pose &rightArmOffset, geometry_msgs::Pose &leftArmOffset, std::vector<double> &rightJointTraj, std::vector<double> &leftJointTraj)
{
    _left_wrist_offset = pose2tf(leftArmOffset);
    _right_wrist_offset = pose2tf(rightArmOffset);
    _obj_pose_traj = objPoseArray;
    if(!_gen_trajectory(rightJointTraj, leftJointTraj))
        return false;
    return true;
}


//Assumes that wrist offsets and obj pose trajectory is set
bool TubeManipulation::Arms::_gen_trajectory(std::vector<double> &right_joint_traj,
                                             std::vector<double> &left_joint_traj)
{
    right_joint_traj.clear();
    left_joint_traj.clear();
    std::vector<double> right_joints_seed, left_joints_seed;
    std::vector<double> right_joint_state, left_joint_state;

    right_joints_seed.resize(_r_jnt_nms.size());
    left_joints_seed.resize(_r_jnt_nms.size());
    //start with somewhere in the middle
    for(int i=0; i<_r_jnt_nms.size(); i++){
        right_joints_seed[i] = (_joint_bounds[_r_jnt_nms[i].c_str()].first + _joint_bounds[_r_jnt_nms[i].c_str()].second)/2;
        left_joints_seed[i] = (_joint_bounds[_l_jnt_nms[i].c_str()].first + _joint_bounds[_l_jnt_nms[i].c_str()].second)/2;
    }

    geometry_msgs::Pose right_pose, left_pose;
    tf::Transform tf_right_wrist, tf_left_wrist, tf_obj;
    if(_obj_pose_traj.poses.empty()){
        ROS_ERROR_NAMED(ARMS_LGRNM,"Object trajectory is empty");
        return false;
    }
    //temporarly set pointer to given object
    //CollisionCheck collision_check(_rh); //new instance just for this function
    /*collision_check_ptr->setAttachedObjPtr(att_obj_ptr);
    collision_check_ptr->setMarkerLifeTime(0.2);
    collision_check_ptr->enableVisualization();
    collision_check_ptr->refreshState();*/

    _collision_check->refreshState();

    for(unsigned int i=0; i<_obj_pose_traj.poses.size(); i++)
    {
        tf_obj = pose2tf(_obj_pose_traj.poses[i]);

        tf_right_wrist = tf_obj*_right_wrist_offset;
        right_pose = tf2pose(tf_right_wrist);

        tf_left_wrist = tf_obj*_left_wrist_offset;
        left_pose = tf2pose(tf_left_wrist);

        if( _get_simple_right_arm_ik(right_pose, right_joint_state, right_joints_seed) &&
                 _get_simple_left_arm_ik(left_pose, left_joint_state, left_joints_seed) &&
                _collision_check->isStateValid(right_joint_state, left_joint_state) )
        {

            for(unsigned int i=0; i<right_joint_state.size(); i++)
            {
                right_joint_traj.push_back(right_joint_state[i]);
                left_joint_traj.push_back(left_joint_state[i]);

                right_joints_seed[i] = right_joint_state[i];  //seeds for next ik call
                left_joints_seed[i] = left_joint_state[i];   //seeds for next ik call
            }
        }
        else{
            return false;
        }
    }
    return true;
}

/*void TubeManipulation::Arms::setAttachedObjPtr(arm_navigation_msgs::AttachedCollisionObject::Ptr attObjPtr){
    _att_obj_ptr.reset();
    _att_obj_ptr = attObjPtr;
    _collision_check->setAttachedObjPtr(_att_obj_ptr);
}*/

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

    for(unsigned int i=0; i<_r_jnt_nms.size(); i++){
        req.start_state.joint_state.position[i] = _right_goal.trajectory.points[1].positions[i];
    }

    req.trajectory.joint_names = _r_jnt_nms;
    req.trajectory.points.resize(_right_goal.trajectory.points.size());

    for(unsigned int i=0; i<_right_goal.trajectory.points.size(); i++){
        req.trajectory.points[i].positions.resize(_r_jnt_nms.size());
    }

    for(unsigned int i=0; i<_right_goal.trajectory.points.size(); i++){
        for(int j=0; j<_r_jnt_nms.size(); j++){
            req.trajectory.points[i].positions[j] = _right_goal.trajectory.points[i].positions[j];
        }
    }

    if(_traj_unnormalizer_client.call(req,res)){
        if(res.error_code.val == res.error_code.SUCCESS){
            _right_goal.trajectory = res.trajectory;
        }
        else{
            ROS_ERROR_NAMED(ARMS_LGRNM,"Requested right trajectory was not filtered. Error code: %d",res.error_code.val);
            return;
        }
    }
    else{
        ROS_ERROR_NAMED(ARMS_LGRNM,"Service call to right filter trajectory failed %s",_traj_unnormalizer_client.getService().c_str());
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

    for(unsigned int i=0; i<_l_jnt_nms.size(); i++){
        req.start_state.joint_state.position[i] = _left_goal.trajectory.points[1].positions[i];
    }

    req.trajectory.joint_names = _l_jnt_nms;
    req.trajectory.points.resize(_left_goal.trajectory.points.size());

    for(unsigned int i=0; i<_left_goal.trajectory.points.size(); i++){
        req.trajectory.points[i].positions.resize(_l_jnt_nms.size());
    }

    for(unsigned int i=0; i<_left_goal.trajectory.points.size(); i++){
        for(int j=0; j<7; j++)
            req.trajectory.points[i].positions[j] = _left_goal.trajectory.points[i].positions[j];
    }

    if(_traj_unnormalizer_client.call(req,res)){
        if(res.error_code.val == res.error_code.SUCCESS){
            _left_goal.trajectory = res.trajectory;
        }
        else{
            ROS_ERROR_NAMED(ARMS_LGRNM,"Requested left trajectory was not filtered. Error code: %d",res.error_code.val);
            ros::shutdown();
            exit(-1);
        }
    }
    else{
        ROS_ERROR_NAMED(ARMS_LGRNM,"Service call to left filter trajectory failed %s",_traj_unnormalizer_client.getService().c_str());
    }
}

/*! \brief Populates trajecty goal variables from double vector.
 *
 *  
 */
void TubeManipulation::Arms::_get_right_goal()
{
    trajectory_msgs::JointTrajectoryPoint traj_point;

    traj_point.positions.resize(_r_jnt_nms.size());
    traj_point.velocities.resize(_r_jnt_nms.size());

    _right_goal.trajectory.joint_names = _r_jnt_nms;
    _right_goal.trajectory.points.resize(_obj_pose_traj.poses.size());

    for(int j=0; j<_r_jnt_nms.size(); j++){
        traj_point.positions[j] = _left_joint_traj[j];
        traj_point.velocities[j] = 0.0;
    }
    traj_point.time_from_start = ros::Duration(0.25);
    _right_goal.trajectory.points[0] = traj_point;

    for(unsigned int i=1; i<_obj_pose_traj.poses.size(); i++){
        for(int j=0; j<_r_jnt_nms.size(); j++){
            traj_point.positions[j] = _left_joint_traj[(i*(_r_jnt_nms.size()))+j];
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

    for(int j=0; j<_l_jnt_nms.size(); j++){
        traj_point.positions[j] = _left_joint_traj[j];
        traj_point.velocities[j] = 0.0;
    }
    traj_point.time_from_start = ros::Duration(0.25);
    _left_goal.trajectory.points[0] = traj_point;

    for(unsigned int i=1; i<_obj_pose_traj.poses.size(); i++){
        for(int j=0; j<_l_jnt_nms.size(); j++){
            traj_point.positions[j] = _left_joint_traj[(i*(_l_jnt_nms.size()))+j];
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
    for(unsigned int i=1; i < _obj_pose_traj.poses.size(); i++){
        max_right_joint_move = 0;
        for(int j=0; j<7; j++){
            double joint_move = fabs(_right_goal.trajectory.points[i].positions[j] -
                                     _right_goal.trajectory.points[i-1].positions[j]);
            if(joint_move > max_right_joint_move) max_right_joint_move = joint_move;
        }

        max_left_joint_move = 0;
        for(int j=0; j<7; j++){
            double joint_move = fabs(_left_goal.trajectory.points[i].positions[j] -
                                     _left_goal.trajectory.points[i-1].positions[j]);
            if(joint_move > max_left_joint_move) max_left_joint_move = joint_move;
        }

        if(max_right_joint_move>max_left_joint_move)
            max_joint_move = max_right_joint_move;
        else
            max_joint_move = max_left_joint_move;

        double seconds = max_joint_move/MAX_JOINT_VEL;
        if(seconds>6.0)
            ROS_WARN_NAMED(ARMS_LGRNM,"max_joint_move: %0.3f, seconds: %0.3f at traj point %d; check wrap arounds in joints",
                     max_joint_move, seconds, i);
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
    if(!right_traj.points.empty()){
        right_arm = true;
        right_goal.trajectory = right_traj;
    }
    if(!left_traj.points.empty()){
        left_arm = true;
        left_goal.trajectory = left_traj;
    }

    //
    ros::Time time_to_start = ros::Time::now()+ros::Duration(1);
    if(right_arm){
        right_goal.trajectory.header.stamp = time_to_start;
        _traj_client_r->sendGoal(right_goal);
    }
    if(left_arm){
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
        ros::Time stop_time = ros::Time::now() + ros::Duration(15);
        while(err>0.01 && ros::Time::now()<stop_time && ros::ok())
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

/*
void TubeManipulation::Arms::_set_planning_scene(void){
    arm_navigation_msgs::SetPlanningSceneDiff::Request req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response res;
    if(_att_obj_ptr){
        if(!_att_obj_ptr->object.shapes.empty()){
            req.planning_scene_diff.attached_collision_objects.push_back(*_att_obj_ptr);
        }
    }
    else{
        ROS_WARN_NAMED(ARMS_LGRNM,"Attached Object Pointer is NULL!");
    }
    if(!_set_pln_scn_client.call(req,res)){
        ROS_ERROR_NAMED(ARMS_LGRNM,"Couldn't set planning scene");
    }
}*/

bool TubeManipulation::Arms::moveRightArmWithMPlanning(std::vector<double> &goalJoints){
    return _move_right_arm_with_mplning(goalJoints);
}

bool TubeManipulation::Arms::moveRightArmWithMPlanning(geometry_msgs::Pose pose)
{
    std::vector<double> ik_joints(7), crnt_joints;
    _get_right_joints(crnt_joints);
    //_set_planning_scene();
    if(!_get_right_arm_ik(pose,ik_joints,crnt_joints)){
        ROS_WARN_NAMED(ARMS_LGRNM,"Right arm IK returned with no solution");
        return false;
    }
    return _move_right_arm_with_mplning(ik_joints);
}

bool TubeManipulation::Arms::_move_right_arm_with_mplning(std::vector<double> &ik_joints)
{
    if(ik_joints.empty() || ik_joints.size()!=_r_jnt_nms.size()){
        ROS_WARN_NAMED(ARMS_LGRNM,"Size of input argument ik_joints is not %d",_r_jnt_nms.size());
        return false;
    }
    _collision_check->refreshState();
    std::vector<double> crnt_right_joints;
    _get_right_joints(crnt_right_joints);

    arm_navigation_msgs::GetMotionPlan::Request req;
    arm_navigation_msgs::GetMotionPlan::Response res;

    req.motion_plan_request.group_name = "right_arm";
    req.motion_plan_request.num_planning_attempts = 3;
    req.motion_plan_request.planner_id = std::string("");
    req.motion_plan_request.allowed_planning_time = ros::Duration(60.0);
    req.motion_plan_request.start_state.joint_state.name = _r_jnt_nms;
    req.motion_plan_request.start_state.joint_state.position = crnt_right_joints;
    req.motion_plan_request.goal_constraints.joint_constraints.resize(_r_jnt_nms.size());
    
    //ROS_INFO_NAMED(ARMS_LGRNM,"Adding following contraints in right arm GetPlan request...");
    for (unsigned int i = 0 ; i < req.motion_plan_request.goal_constraints.joint_constraints.size(); i++)
    {
      req.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = _r_jnt_nms[i];
      req.motion_plan_request.goal_constraints.joint_constraints[i].position = ik_joints[i];
      req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.02;
      req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.02;
      //ROS_INFO_STREAM_NAMED(ARMS_LGRNM,"Joint - Name : "<<_r_jnt_nms[i].c_str()<<"\tValue : "<<ik_joints[i]);
    }

    if(_start_is_goal(req,res)){
        ROS_INFO_NAMED(ARMS_LGRNM,"Current state already is goal state. Skipping motion planning");
        return true;
    }
    int nr_try = 3;
    do{
        if(!_get_motion_plan(req,res)){
            _handle_planning_error(req, res);
        }
        else{
            break;
        }
        nr_try--;
    }while(nr_try>0);
    if(nr_try<=0)
        return false;
    trajectory_msgs::JointTrajectory mplan_traj, filtered_traj, empty_traj;
    mplan_traj = res.trajectory.joint_trajectory;
    if(!_filter_trajectory(mplan_traj, filtered_traj, req))
        return false;
    ROS_DEBUG_NAMED(ARMS_LGRNM,"Trajectory is filtered");
    _execute_joint_trajectory(filtered_traj,empty_traj);
    ROS_DEBUG_NAMED(ARMS_LGRNM,"Executing joint trajectory");
    return true;
}

bool TubeManipulation::Arms::moveLeftArmWithMPlanning(std::vector<double> &goalJoints){
    return _move_left_arm_with_mplning(goalJoints);
}

bool TubeManipulation::Arms::moveLeftArmWithMPlanning(geometry_msgs::Pose pose)
{
    std::vector<double> ik_joints(7), crnt_joints;
    _get_left_joints(crnt_joints);
    //_set_planning_scene();
    if(!_get_left_arm_ik(pose,ik_joints,crnt_joints)){
        ROS_WARN_NAMED(ARMS_LGRNM,"Left arm IK returned with no solution");
        return false;
    }
    return _move_left_arm_with_mplning(ik_joints);
}

bool TubeManipulation::Arms::_move_left_arm_with_mplning(std::vector<double> &ik_joints)
{
    if(ik_joints.empty() || ik_joints.size()!=_l_jnt_nms.size()){
        ROS_WARN_NAMED(ARMS_LGRNM,"Size of input argument ik_joints is not %d",_l_jnt_nms.size());
        return false;
    }

    _collision_check->refreshState();

    std::vector<double> crnt_left_joints;
    _get_left_joints(crnt_left_joints);

    arm_navigation_msgs::GetMotionPlan::Request req;
    arm_navigation_msgs::GetMotionPlan::Response res;

    req.motion_plan_request.group_name = "left_arm";
    req.motion_plan_request.num_planning_attempts = 3;
    req.motion_plan_request.planner_id = std::string("");
    req.motion_plan_request.allowed_planning_time = ros::Duration(60.0);
    req.motion_plan_request.start_state.joint_state.name = _l_jnt_nms;
    req.motion_plan_request.start_state.joint_state.position = crnt_left_joints;
    req.motion_plan_request.goal_constraints.joint_constraints.resize(_l_jnt_nms.size());

    //ROS_INFO_NAMED(ARMS_LGRNM,"Adding following contraints in left arm GetPlan request...");
    for (unsigned int i = 0 ; i < req.motion_plan_request.goal_constraints.joint_constraints.size(); i++)
    {
      req.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = _l_jnt_nms[i];
      req.motion_plan_request.goal_constraints.joint_constraints[i].position = ik_joints[i];
      req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.02;
      req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.02;
      //ROS_INFO_STREAM_NAMED(ARMS_LGRNM,"Joint - Name : "<<_l_jnt_nms[i].c_str()<<"\tValue : "<<ik_joints[i]);
    }

    if(_start_is_goal(req,res)){
        ROS_INFO_NAMED(ARMS_LGRNM,"Current state already is goal state. Skipping motion planning");
        return true;
    }

    if(!_get_motion_plan(req,res)){
        if(!_handle_planning_error(req, res)){
            return false;
        }
    }

    trajectory_msgs::JointTrajectory mplan_traj, filtered_traj, empty_traj;
    mplan_traj = res.trajectory.joint_trajectory;
    if(!_filter_trajectory(mplan_traj, filtered_traj, req))
        return false;
    ROS_DEBUG_NAMED(ARMS_LGRNM,"Trajectory is filtered");
    _execute_joint_trajectory(empty_traj, filtered_traj);
    ROS_DEBUG_NAMED(ARMS_LGRNM,"Executing joint trajectory");
    return true;
}

bool TubeManipulation::Arms::_start_is_goal(arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res)
{
    bool right_arm = false, left_arm = false;
    if(req.motion_plan_request.group_name.compare("right_arm")==0)
        right_arm = true;
    else if(req.motion_plan_request.group_name.compare("left_arm")==0)
        left_arm = true;
    else{
        ROS_ERROR_NAMED(ARMS_LGRNM,"invalid group name");
        return false;
    }

    std::vector<double> right_joints, left_joints;
    _get_right_joints(right_joints);
    _get_left_joints(left_joints);

    double err = 0;
    if(right_arm){
        for(int i=0; i<_r_jnt_nms.size(); i++){
            err += req.motion_plan_request.goal_constraints.joint_constraints[i].position - right_joints[i];
        }
        //ROS_WARN_NAMED(ARMS_LGRNM,"Right error = %f",err);
        if(std::abs(err)<0.001)
            return true;
    }
    err = 0;
    if(left_arm){
        for(int i=0; i<_l_jnt_nms.size(); i++){
            err += req.motion_plan_request.goal_constraints.joint_constraints[i].position - left_joints[i];
        }
        //ROS_WARN_NAMED(ARMS_LGRNM,"Left error = %f",err);
        if(std::abs(err)<0.001)
            return true;
    }
    return false;
}

bool TubeManipulation::Arms::_handle_planning_error(arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res)
{
    bool right_arm = false, left_arm = false;
    if(req.motion_plan_request.group_name.compare("right_arm")==0){
        right_arm = true;
    }
    if(req.motion_plan_request.group_name.compare("left_arm")==0){
        left_arm = true;
    }

    switch(res.error_code.val){
    case -3 : //START_STATE_IN_COLLISION
    {
        ROS_INFO("Trying to resolve error -3...");
        double box_size = 0.1; //box dimentions are box_size x box_size x box_size
        ROS_INFO_NAMED(ARMS_LGRNM,
                       "Planner says start state is in collision. Choosing another start state within %f x %f x %f box",
                       box_size, box_size, box_size);
        geometry_msgs::Pose pose;
        std::vector<double> ik_soln, crnt_joints;
        if(right_arm){
            pose = getRightArmFK();
            _get_right_joints(crnt_joints);
        }
        else if(left_arm){
            pose = getLeftArmFK();
            _get_left_joints(crnt_joints);
        }
        else{
            ROS_WARN_NAMED(ARMS_LGRNM,"group name in motion planning request doesn't match any arm group");
            return false;
        }
        double x_err, y_err, z_err;
        double cnt = 100;
        do{
            x_err = ((double)rand()/(double)RAND_MAX);
            y_err = ((double)rand()/(double)RAND_MAX);
            z_err = ((double)rand()/(double)RAND_MAX);
            pose.position.x += x_err * box_size;
            pose.position.y += y_err * box_size;
            pose.position.z += z_err * box_size;
            if(right_arm){
                if(_get_right_arm_ik(pose, ik_soln, crnt_joints)){
                    req.motion_plan_request.start_state.joint_state.position = ik_soln;
                    //arm_navigation_msgs::GetMotionPlan::Response res_foo;
                }
            }
            else{
                if(_get_left_arm_ik(pose, ik_soln, crnt_joints)){
                    req.motion_plan_request.start_state.joint_state.position = ik_soln;
                    //arm_navigation_msgs::GetMotionPlan::Response res_foo;
                    //res = res_foo;
                }
            }
            cnt--;
        }while(cnt>0);
        ROS_WARN_NAMED(ARMS_LGRNM,"Couldn't solve!");
        break;
    }

    case -21 : //JOINT_LIMITS_VIOLATED
    {
        ROS_INFO_NAMED(ARMS_LGRNM,"Trying to resolve error -21...");
        if(right_arm){
            double adj_err = 0.001;
            std::vector<double> crnt_joints;
            _get_right_joints(crnt_joints);
            double requested_val, lower_bound, upper_bound, crnt_val;
            for(int i=0; i<_r_jnt_nms.size(); i++)
            {
                requested_val = req.motion_plan_request.goal_constraints.joint_constraints[i].position;
                crnt_val = crnt_joints[i];
                lower_bound = _joint_bounds[_r_jnt_nms[i].c_str()].first;
                upper_bound = _joint_bounds[_r_jnt_nms[i].c_str()].second;


                ROS_INFO_STREAM_NAMED(ARMS_LGRNM," "<<_r_jnt_nms[i].c_str()
                                      <<"  Bounds = ["<<lower_bound<<"  "<<upper_bound
                                <<"]  Requested value : "<<requested_val
                                      <<"  Current value : "<<crnt_val);
                if(crnt_val<lower_bound){
                    ROS_WARN_STREAM_NAMED(ARMS_LGRNM,"Current joint value "
                                          <<requested_val<<" of "<<_r_jnt_nms[i].c_str()
                                          <<" joint is out of lower bound");
                }
                if(crnt_val>upper_bound){
                    ROS_WARN_STREAM_NAMED(ARMS_LGRNM,"Current joint value "
                                          <<requested_val<<" of "<<_r_jnt_nms[i].c_str()
                                          <<" joint is out of upper bound");
                }
                if(crnt_val<lower_bound){
                    double err = lower_bound - crnt_val;
                    if(err<adj_err){
                        ROS_WARN_STREAM_NAMED(ARMS_LGRNM,"Joint "<<_r_jnt_nms[i]
                                              <<" is out of lower bound. Adding "<<adj_err<<" error to adjust");
                        crnt_val += adj_err;
                        req.motion_plan_request.start_state.joint_state.position[i] = crnt_val;
                    }
                    else{
                        ROS_WARN_STREAM_NAMED(ARMS_LGRNM,"Joint "<<_r_jnt_nms[i]
                                              <<" is out of lower bound. Error value is "
                                              <<err<<">"<<adj_err<<" Can not adjust!");
                    }
                }
                if(crnt_val>upper_bound){
                    double err = crnt_val - upper_bound;
                    if(err<adj_err){
                        ROS_WARN_STREAM_NAMED(ARMS_LGRNM,"Joint "<<_r_jnt_nms[i]
                                              <<" is out of upper bound. Subtracting "<<adj_err<<" error to adjust");
                        crnt_val -= adj_err;
                        req.motion_plan_request.start_state.joint_state.position[i] = crnt_val;
                    }
                    else{
                        ROS_WARN_STREAM_NAMED(ARMS_LGRNM,"Joint "<<_r_jnt_nms[i]
                                              <<" is out of upper bound. Error value is "
                                              <<err<<">"<<adj_err<<" Can not adjust!");
                    }
                }
            }
        }
        if(left_arm){
            ;
        }
        //_get_motion_plan(req,res);
        break;
    }

    case -5 : //GOAL_IN_COLLISION
    {
        ROS_INFO_NAMED(ARMS_LGRNM,"Trying to resolving error -5...");
        _collision_check->refreshState();
        //_collision_check->setAttachedObj(attObj);
        _collision_check->enableVisualization();
        _collision_check->setMarkerLifeTime(30);
        std::vector<double> right_joints, left_joints;
        if(right_arm){
            for(int i=0; i<_r_jnt_nms.size(); i++){
                right_joints[i] = req.motion_plan_request.goal_constraints.joint_constraints[i].position;
            }
            _get_left_joints(left_joints);
        }
        if(left_arm){
            for(int i=0; i<_l_jnt_nms.size(); i++){
                left_joints[i] = req.motion_plan_request.goal_constraints.joint_constraints[i].position;
            }
            _get_right_joints(right_joints);
        }

        if(!_collision_check->isStateValid(right_joints, left_joints)){
            ROS_WARN_STREAM_NAMED(ARMS_LGRNM,"State validity error : "<<_collision_check->getLastErrorAsString());
            int err = _collision_check->getLastError();
            if(err==_collision_check->OUT_OF_BOUND_L ||
                    err==_collision_check->OUT_OF_BOUND_R){
                ROS_ERROR_NAMED(ARMS_LGRNM,"TODO: IF JOINTS ARE OUT OF BOUND");
                //TODO:
            }
            if(err==_collision_check->IN_ENV_CLSN_R ||
                    err==_collision_check->IN_ENV_CLSN_L ||
                    err==_collision_check->IN_SLF_CLSN_R ||
                    err==_collision_check->IN_SLF_CLSN_L ){
                ROS_ERROR_NAMED(ARMS_LGRNM,"TODO: IF GOAL STATE IS IN COLLISION");
                //TODO:
            }
        }
        break;
    }

    default :
    {
        ROS_ERROR_NAMED(ARMS_LGRNM,"Could not resolve error %d. Case handeling for this error is not implemented yet",res.error_code.val);
        return false;
    }
    }//switch
    return false;
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
        ros::Time stop_time = ros::Time::now() + ros::Duration(15);
        while(err>0.01 && ros::Time::now()<stop_time && ros::ok())
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
            goal.velocities[j] = MAX_VELOCITY;
        }
        goal.time_from_start = ros::Duration(0.0);
        traj_goal.trajectory.points[0] = goal;
        ros::Time time_to_start = ros::Time::now()+ros::Duration(0.1);
        traj_goal.trajectory.header.stamp = time_to_start;
        _traj_client_r->sendGoalAndWait(traj_goal, ros::Duration(10));
        std::vector<double> joints(_r_jnt_nms.size());
        double err = 1;
        ros::Time stop_time = ros::Time::now() + ros::Duration(15);
        while(err>0.01 && ros::Time::now()<stop_time && ros::ok())
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
            goal.velocities[j] = MAX_VELOCITY;
        }
        goal.time_from_start = ros::Duration(0.0);
        traj_goal.trajectory.points[0] = goal;
        ros::Time time_to_start = ros::Time::now()+ros::Duration(0.1);
        traj_goal.trajectory.header.stamp = time_to_start;
        _traj_client_l->sendGoalAndWait(traj_goal,ros::Duration(10));
        std::vector<double> joints(_l_jnt_nms.size());
        double err = 1;
        ros::Time stop_time = ros::Time::now() + ros::Duration(15);
        while(err>0.01 && ros::Time::now()<stop_time && ros::ok())
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

bool TubeManipulation::Arms::getSimpleLeftArmIK(geometry_msgs::Pose pose,
                             std::vector<double> &jointsOut)
{
    std::vector<double> joints(7);
    _get_left_joints(joints);
    return(_get_simple_left_arm_ik(pose, jointsOut, joints));
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
    /*kinematics_msgs::GetConstraintAwarePositionIK::Request  ik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;

    ik_req.timeout = ros::Duration(5.0);
    ik_req.ik_request.ik_link_name = "r_wrist_roll_link";
    ik_req.ik_request.ik_seed_state.joint_state.name = _r_jnt_nms;
    ik_req.ik_request.ik_seed_state.joint_state.position.resize(_r_jnt_nms.size());
    if(seed_state.size()==_r_jnt_nms.size()){
        ik_req.ik_request.ik_seed_state.joint_state.position = seed_state;
    }
    else{
        ROS_ERROR_NAMED(ARMS_LGRNM,"Seed state value is less than number of joints (%d)",_r_jnt_nms.size());
        return 0;
    }

    ik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
    ik_req.ik_request.pose_stamped.pose = pose;

    if(_ik_client_r.call(ik_req, ik_res)){
      if(ik_res.error_code.val == ik_res.error_code.SUCCESS){
          joint_state = ik_res.solution.joint_state;
      }
      else if(ik_res.error_code.val != ik_res.error_code.NO_IK_SOLUTION){
          ROS_INFO_STREAM_NAMED(ARMS_LGRNM,"Right Arm Ik error code : "<<ik_res.error_code.val
                          <<" ("<<arm_navigation_msgs::armNavigationErrorCodeToString(ik_res.error_code)<<")");
          return false;
      }
      else
        return false;
    }
    else{
      ROS_ERROR_NAMED(ARMS_LGRNM,"Right arm Inverse kinematics service call failed.");
      return false;
    }*/
    if(!_get_simple_right_arm_ik(pose,joint_state,seed_state)){
        return false;
    }
    _collision_check->refreshState();
    std::vector<double> joints(_r_jnt_nms.size());
    joints = joint_state.position;
    if(!_collision_check->isRightArmStateValid(joints)){
        ROS_INFO_STREAM_NAMED(ARMS_LGRNM,"Collision Error : "<<_collision_check->getLastErrorAsString());
        return false;
    }
    return true;
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
    if(seed_state.size()==_r_jnt_nms.size()){
        for(unsigned int i=0; i<_r_jnt_nms.size(); i++)
            ik_req.ik_request.ik_seed_state.joint_state.position[i] = seed_state[i];
    }
    else{
        ROS_ERROR_NAMED(ARMS_LGRNM,"Seed state value is less than number of joints (%d)", _r_jnt_nms.size());
        return 0;
    }

    ik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
    ik_req.ik_request.pose_stamped.pose = pose;

    if(_smpl_ik_client_r.call(ik_req, ik_res)){
      if(ik_res.error_code.val == ik_res.error_code.SUCCESS){
          joint_state = ik_res.solution.joint_state;
      }
      else{
        ROS_DEBUG_NAMED(ARMS_LGRNM,"Right arm Inverse kinematics failed for given pose.");
        return 0;
      }
    }
    else{
      ROS_ERROR_NAMED(ARMS_LGRNM,"Right arm Inverse kinematics service call failed.");
      return 0;
    }
    return 1;
}

bool TubeManipulation::Arms::_get_left_arm_ik(geometry_msgs::Pose pose,
                                sensor_msgs::JointState &joint_state,
                                std::vector<double> &seed_state)
{
    /*kinematics_msgs::GetConstraintAwarePositionIK::Request  ik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;

    ik_req.timeout = ros::Duration(5.0);
    ik_req.ik_request.ik_link_name = "l_wrist_roll_link";
    ik_req.ik_request.ik_seed_state.joint_state.name = _l_jnt_nms;
    ik_req.ik_request.ik_seed_state.joint_state.position.resize(_l_jnt_nms.size());

    if(seed_state.size()==_l_jnt_nms.size()){
        ik_req.ik_request.ik_seed_state.joint_state.position = seed_state;
    }
    else{
        ROS_ERROR_NAMED(ARMS_LGRNM,"Seed state value is less than number of joints (7)");
        return 0;
    }
    ik_req.ik_request.pose_stamped.header.frame_id = "base_link";
    ik_req.ik_request.pose_stamped.pose = pose;

    if(_ik_client_l.call(ik_req, ik_res)){
      if(ik_res.error_code.val == ik_res.error_code.SUCCESS){
          joint_state = ik_res.solution.joint_state;
      }
      else if(ik_res.error_code.val != ik_res.error_code.NO_IK_SOLUTION){
          ROS_INFO_STREAM_NAMED(ARMS_LGRNM,"Left Arm Ik error code : "<<ik_res.error_code.val
                          <<" ("<<arm_navigation_msgs::armNavigationErrorCodeToString(ik_res.error_code)<<")");
        return false;
      }
      else
          return false;
    }
    else{
      ROS_ERROR_NAMED(ARMS_LGRNM,"Left arm Inverse kinematics service call failed.");
      return false;
    }
    return true;*/

    if(!_get_simple_left_arm_ik(pose,joint_state,seed_state)){
        return false;
    }
    _collision_check->refreshState();
    std::vector<double> joints(_l_jnt_nms.size());
    joints = joint_state.position;
    if(!_collision_check->isLeftArmStateValid(joints)){
        ROS_INFO_STREAM_NAMED(ARMS_LGRNM,"Collision Error : "<<_collision_check->getLastErrorAsString());
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
    if(seed_state.size()==_l_jnt_nms.size()){
        for(unsigned int i=0; i<_l_jnt_nms.size(); i++)
            ik_req.ik_request.ik_seed_state.joint_state.position[i] = seed_state[i];
    }
    else{
        ROS_ERROR_NAMED(ARMS_LGRNM,"Seed state value is less than number of joints (%d)", _l_jnt_nms.size());
        return 0;
    }

    ik_req.ik_request.pose_stamped.header.frame_id = "/base_link";
    ik_req.ik_request.pose_stamped.pose = pose;

    if(_smpl_ik_client_l.call(ik_req, ik_res)){
      if(ik_res.error_code.val == ik_res.error_code.SUCCESS){
          joint_state = ik_res.solution.joint_state;
      }
      else{
        ROS_DEBUG_NAMED(ARMS_LGRNM,"Left arm Inverse kinematics failed for given pose.");
        return false;
      }
    }
    else{
      ROS_ERROR_NAMED(ARMS_LGRNM,"Left arm Inverse kinematics service call failed.");
      return false;
    }
    return true;
}

//for object held by right arm
bool TubeManipulation::Arms::getRegraspPoseRight(arm_navigation_msgs::AttachedCollisionObject::Ptr attObjPtr,
                                                 geometry_msgs::Pose crnt_grasp, //current grasp in tube frame
                                                 geometry_msgs::Pose wrist_pose, //current wrist pose
                                                 geometry_msgs::Pose other_hand_grasp, //left hand desired grasp
                                                 geometry_msgs::Pose &obj_pose_out,  //new object pose
                                                 std::vector<double> &ik_soln) //incase if ik fails to move to new pose
{
    bool pose_found = false;
    TubeManipulation::CollisionCheck collision_check(_rh, _collision_objects);
    collision_check.enableVisualization();
    //collision_check.setAttachedObjPtr(attObjPtr);
    collision_check.refreshState();
    _collision_objects->printListOfObjects();
    tf::Transform obj_orig, obj, rand_tf,
            wrist=pose2tf(wrist_pose),
            grasp=pose2tf(crnt_grasp),
            other_grasp = pose2tf(other_hand_grasp);

    obj_orig = wrist * grasp.inverse();

    tf::Quaternion q;
    tf::Vector3 pos;
    double y,p,r;
    double x_pos, y_pos, z_pos;
    int MAX_CNT = 10000;
    int cnt = MAX_CNT;
    obj = obj_orig; //start with object's original pose
    std::vector<double> right_joints(7), right_seeds(7),
                        left_seeds(7), left_joints(7);

    _get_right_joints(right_seeds);
    _get_left_joints(left_seeds);
    tf::Transform wrist_crnt, other_wrist;
    geometry_msgs::Pose wrist_crnt_pose, other_wrist_pose;

    ROS_INFO_NAMED(ARMS_LGRNM,"Checking %d Object poses for regrasp...",cnt);
    do
    {
        wrist_crnt = obj * grasp;
        other_wrist = obj * other_grasp;
        wrist_crnt_pose = tf2pose(wrist_crnt);
        other_wrist_pose = tf2pose(other_wrist);
        if(_get_simple_right_arm_ik(wrist_crnt_pose, right_joints, right_seeds)
           &&_get_simple_left_arm_ik(other_wrist_pose, left_joints, left_seeds) )
        {
            if(collision_check.isStateValid(right_joints, left_joints))
            {
                collision_check.setMarkerLifeTime(60);
                collision_check.isStateValid(right_joints, left_joints);
                sleep(2);
                ik_soln = left_joints;
                std::cout<<std::endl;
                ROS_INFO_NAMED(ARMS_LGRNM,"Valid object pose found in %d iteration", (MAX_CNT-cnt));
                //cnt = 0;
                obj_orig = obj;
                pose_found = true;
                break;
            }
        }

        //random orientation
        y = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
        p = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
        r = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
        q.setRPY(r,p,y);

        // bounding box to move around
        double x_min = -0.25,
                x_max = 0.25,
                y_min = -0.2,
                y_max = 0.2,
                z_min = 0.1,
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

        rand_tf.setOrigin(tf::Vector3(0,0,0));
        rand_tf.setRotation(q);
        //rand_tf.setOrigin(pos);

        // move around in box. Note that pos is in global frame orientation
        obj = obj_orig * rand_tf;
        tf::Vector3 obj_pose = obj.getOrigin();
        obj_pose += pos;
        obj.setOrigin(obj_pose);

        /*tf::Vector3 vec = obj.getOrigin();
        std::cout<<"["<<vec.getX()<<"  "<<vec.getY()<<"  "<<vec.getZ()<<"]";*/
        std::cout<<'\r'<<cnt<<' '<<std::flush;
        cnt--;
        }while(cnt>1);
    if(cnt==0)
        std::cout<<'\n';
    obj_pose_out = tf2pose(obj_orig);

    if(!pose_found)
        ROS_WARN_NAMED(ARMS_LGRNM,"Couldn't find valid object pose for regrasping");

    return pose_found;
}

void TubeManipulation::Arms::_publish_pose(const geometry_msgs::Pose &pose){
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "/base_link";
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose = pose;
    _test_pose_pub.publish(pose_stamped);
}


bool TubeManipulation::Arms::moveRightArmToCloudCapturePose(const tf::Vector3 &sensorOrig,
                                                         const tf::Vector3 &startOrig,
                                                         const double minSensorDistance,
                                                         std::vector<double> &ikSoln){
    ros::Publisher vis_msg = _rh->advertise<visualization_msgs::MarkerArray>("tube_polishing/arms/arrows",1);
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;
//    _collision_check->refreshState();

//    std::vector<double> crnt_joints;
//    _get_right_joints(crnt_joints);

//    arm_navigation_msgs::GetMotionPlan::Request req;
//    arm_navigation_msgs::GetMotionPlan::Response res;

//    req.motion_plan_request.group_name = "right_arm";
//    req.motion_plan_request.num_planning_attempts = 3;
//    req.motion_plan_request.planner_id = std::string("");
//    req.motion_plan_request.allowed_planning_time = ros::Duration(60.0);
//    req.motion_plan_request.start_state.joint_state.name = _r_jnt_nms;
//    req.motion_plan_request.start_state.joint_state.position = crnt_joints;

//    arm_navigation_msgs::PositionConstraint pos_cnstr;
//    arm_navigation_msgs::OrientationConstraint orie_cnstr;

//    pos_cnstr.header.frame_id = "/base_link";
//    pos_cnstr.header.stamp = ros::Time::now();
//    pos_cnstr.link_name = "r_wrist_roll_link";
//    pos_cnstr.constraint_region_orientation.w = 1.0;
//    pos_cnstr.constraint_region_shape.type = pos_cnstr.constraint_region_shape.BOX;
//    pos_cnstr.constraint_region_shape.dimensions.push_back(0.25);
//    pos_cnstr.constraint_region_shape.dimensions.push_back(0.25);
//    pos_cnstr.constraint_region_shape.dimensions.push_back(0.25);
//    pos_cnstr.position.x = 0.5;
//    pos_cnstr.position.y = 0.0;
//    pos_cnstr.position.z = 0.7;
//    pos_cnstr.weight = 1.0;

//    tf::Quaternion q(0.0, 0.0, 0.0, 1.0);
//    q.setRotation(tf::Vector3(0.0, 0.0, 1.0), M_PI);
//    orie_cnstr.header.frame_id = "/base_link";
//    orie_cnstr.header.stamp = ros::Time::now();
//    orie_cnstr.link_name = "r_wrist_roll_link";
//    orie_cnstr.type = orie_cnstr.HEADER_FRAME;
//    orie_cnstr.orientation.x = q.getX();
//    orie_cnstr.orientation.y = q.getY();
//    orie_cnstr.orientation.z = q.getZ();
//    orie_cnstr.orientation.w = q.getW();
//    orie_cnstr.weight = 1.0;
//    orie_cnstr.absolute_pitch_tolerance = 0.02;
//    orie_cnstr.absolute_yaw_tolerance = 0.02;
//    orie_cnstr.absolute_roll_tolerance = 2*M_PI;

//    req.motion_plan_request.goal_constraints.orientation_constraints.push_back(orie_cnstr);
//    req.motion_plan_request.goal_constraints.position_constraints.push_back(pos_cnstr);

//    //req.motion_plan_request.goal_constraints.joint_constraints.resize(_l_jnt_nms.size());

//    //ROS_INFO_NAMED(ARMS_LGRNM,"Adding following contraints in left arm GetPlan request...");
//    /*for (unsigned int i = 0 ; i < req.motion_plan_request.goal_constraints.joint_constraints.size(); i++)
//    {
//      req.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = _l_jnt_nms[i];
//      req.motion_plan_request.goal_constraints.joint_constraints[i].position = ik_joints[i];
//      req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.02;
//      req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.02;
//      //ROS_INFO_STREAM_NAMED(ARMS_LGRNM,"Joint - Name : "<<_l_jnt_nms[i].c_str()<<"\tValue : "<<ik_joints[i]);
//    }*/

//    /*if(_start_is_goal(req,res)){
//        ROS_INFO_NAMED(ARMS_LGRNM,"Current state already is goal state. Skipping motion planning");
//        return true;
//    }*/

//    if(!_get_motion_plan(req,res)){
//        if(!_handle_planning_error(req, res)){
//            return false;
//        }
//    }

//    trajectory_msgs::JointTrajectory mplan_traj, filtered_traj, empty_traj;
//    mplan_traj = res.trajectory.joint_trajectory;
//    if(!_filter_trajectory(mplan_traj, filtered_traj, req))
//        return false;
//    ROS_DEBUG_NAMED(ARMS_LGRNM,"Trajectory is filtered");
//    _execute_joint_trajectory(empty_traj, filtered_traj);
//    ROS_DEBUG_NAMED(ARMS_LGRNM,"Executing joint trajectory");
//    return true;


    //*************************************************************************

    TubeManipulation::CollisionCheck collision_check(_rh, _collision_objects);
    collision_check.enableVisualization();
    collision_check.setMarkerLifeTime(10);
    //collision_check.setAttachedObjPtr(attObjPtr);
    collision_check.refreshState();
    //_collision_objects->printListOfObjects();

    tf::Transform rand_pose;
    tf::Vector3 ray_to_sensor, x_axis;

    double y, p, r;
    tf::Quaternion q;
    tf::Vector3 rand_orig, perp_vec;
    std::vector<double> ik_soln(_r_jnt_nms.size()), current_joints;
    _get_right_joints(current_joints);
    geometry_msgs::Pose wrist_pose;
    double min_dist_sq = minSensorDistance * minSensorDistance;
    int cnt = 10000;
    bool pose_found = false;
    do{
//        ma.markers.clear();
        // bounding box to move around
        double x_min = 0.4,
                x_max = 0.8,
                y_min = -0.35,
                y_max = 0.1,
                z_min = 0.7,
                z_max = 1.5;

        // random number between 0 and 1
        double x_pos = ((double)rand()/(double)RAND_MAX);
        double y_pos = ((double)rand()/(double)RAND_MAX);
        double z_pos = ((double)rand()/(double)RAND_MAX);

        double len;
        len = x_max - x_min;
        x_pos = x_min + (len * x_pos);
        len = y_max - y_min;
        y_pos = y_min + (len * y_pos);
        len = z_max - z_min;
        z_pos = z_min + (len * z_pos);

        rand_orig.setValue(x_pos, y_pos, z_pos);
        //rand_orig += startOrig;

        ray_to_sensor = sensorOrig - rand_orig;
        //std::cout<<"\nDist "<<ray_to_sensor.length();
        if(ray_to_sensor.length()<minSensorDistance){
            //std::cout<<"\nDist "<<ray_to_sensor.length();
            cnt--;
            continue; //continue before cnt--
        }

//        m.action = m.ADD;
//        m.color.r = 1;
//        m.color.a = 1;
//        m.header.frame_id = "/base_link";
//        m.header.stamp = ros::Time::now();
//        m.id = 1;
//        m.type = m.ARROW;
//        //m.lifetime = ros::Duration(1);

//        m.scale.x = 0.005;
//        m.scale.y = 0.006;
//        m.scale.z = 0.01;

//        geometry_msgs::Point point;
//        point.x = rand_orig.getX();
//        point.y = rand_orig.getY();
//        point.z = rand_orig.getZ();
//        m.points.push_back(point);
//        point.x += ray_to_sensor.getX();
//        point.y += ray_to_sensor.getY();
//        point.z += ray_to_sensor.getZ();
//        m.points.push_back(point);

//        ma.markers.push_back(m);

        /*y = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
        p = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;*/
        r = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
        q.setValue(0,0,0,1);
        q.setRotation(tf::Vector3(0,0,1),M_PI);
        tf::Transform x_tf;
        x_tf.setIdentity();
        x_tf.setRotation(q);

        //q.setRPY(r,p,y);

        //rand_pose.setIdentity();
        //rand_pose.setRotation(q);
        //get x axis vector
        //rand_pose = rand_pose * tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(1.0, 0.0, 0.0));
        //x_axis = rand_pose.getOrigin();
        x_axis.setValue(1,0,0); //x axis of q is negative x of global frame

//        m.id++;
//        m.points.clear();
//        point.x = rand_orig.getX();
//        point.y = rand_orig.getY();
//        point.z = rand_orig.getZ();
//        m.points.push_back(point);
//        point.x += 1;
//        m.points.push_back(point);
//        ma.markers.push_back(m);

        double angle = ray_to_sensor.angle(x_axis);
        //std::cout<<"\nTheta : "<<angle;
        perp_vec = x_axis.cross(ray_to_sensor);
//        m.id++;
//        m.points.clear();
//        point.x = rand_orig.getX();
//        point.y = rand_orig.getY();
//        point.z = rand_orig.getZ();
//        m.points.push_back(point);
//        point.x += perp_vec.getX();
//        point.y += perp_vec.getY();
//        point.z += perp_vec.getZ();
//        m.points.push_back(point);
//        ma.markers.push_back(m);

        q.setRotation(perp_vec, angle);

        rand_pose.setRotation(q);
        rand_pose.setOrigin(rand_orig);

        tf::Transform theta_tf;
        theta_tf.setIdentity();
        q = tf::Quaternion::getIdentity();
        q.setRotation(tf::Vector3(1,0,0),r);
        theta_tf.setRotation(q);

        //rand_pose *= theta_tf;

        //double rand_angle = ((double)rand()/(double)RAND_MAX)*2*M_PI;
        //q.setRotation(tf::Vector3(1,0,0),rand_angle);

        //rand_pose.setOrigin(rand_orig);
        //rand_pose.setRotation(q);
        wrist_pose = tf2pose(rand_pose);
        _publish_pose(wrist_pose);
        if(_get_simple_right_arm_ik(wrist_pose, ik_soln, current_joints)){
            if(collision_check.isRightArmStateValid(ik_soln)){
                ros::Duration(3).sleep();
                cnt = 0;
                ikSoln = ik_soln;
                ROS_INFO_NAMED(ARMS_LGRNM, "Capture pose found");
                pose_found = true;
            }
        }
        cnt--;
//        vis_msg.publish(ma);
//        while(ros::ok())
//        {
//            std::string s;
//            std::cin>>s;
//        }
        ros::Duration(0.05).sleep();
    }while(cnt>1);

    return pose_found;
}

//for object held by left hand
bool TubeManipulation::Arms::getRegraspPoseLeft(arm_navigation_msgs::AttachedCollisionObject::Ptr attObjPtr,
                                                geometry_msgs::Pose crnt_grasp,
                                                     geometry_msgs::Pose wrist_pose,
                                                     geometry_msgs::Pose other_hand_grasp,
                                                     geometry_msgs::Pose &obj_pose_out,
                                                    std::vector<double> &ik_soln)
{
    bool pose_found = false;
    TubeManipulation::CollisionCheck collision_check(_rh, _collision_objects);
    collision_check.enableVisualization();
    //collision_check.setAttachedObjPtr(attObjPtr);
    tf::Transform obj_orig, obj, rand_tf,
            wrist=pose2tf(wrist_pose),
            grasp=pose2tf(crnt_grasp),
            other_grasp = pose2tf(other_hand_grasp);

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

    ROS_INFO_NAMED(ARMS_LGRNM,"Checking %d Object poses for regrasp...",cnt);
    do
    {

        wrist_crnt = obj * grasp;
        other_wrist = obj * other_grasp;
        wrist_crnt_pose = tf2pose(wrist_crnt);
        other_wrist_pose = tf2pose(other_wrist);
        if(_get_simple_left_arm_ik(wrist_crnt_pose, left_joints, left_seeds)
           &&_get_simple_right_arm_ik(other_wrist_pose, right_joints, right_seeds) )
        {
            if(collision_check.isStateValid(right_joints, left_joints))
            {
                collision_check.setMarkerLifeTime(60);
                collision_check.isStateValid(right_joints, left_joints);
                sleep(10);
                ik_soln = right_joints;
                ROS_INFO_NAMED(ARMS_LGRNM,"Valid object pose found in %d iteration", (MAX_CNT-cnt));
                cnt = 0;
                obj_orig = obj;
                pose_found = true;
            }
        }

        y = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
        p = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
        r = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
        q.setRPY(r,p,y);

        // bounding box to move around
        double x_min = 0,
                x_max = 0,
                y_min = -0.25,
                y_max = 0.25,
                z_min = 0.1,
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
        std::cout<<'\r'<<cnt<<std::flush;
        }while(cnt>0);
    std::cout<<'\n';
    obj_pose_out = tf2pose(obj_orig);

    if(!pose_found)
        ROS_WARN_NAMED(ARMS_LGRNM,"Couldn't find valid object pose for regrasping");

    return pose_found;
}


bool TubeManipulation::Arms::_get_motion_plan(arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res)
{
    while(!ros::service::waitForService("ompl_planning/plan_kinematic_path",1))
        ROS_INFO_NAMED(ARMS_LGRNM,"waiting for ompl_planning/plan_kinematic_path");
    ros::ServiceClient srv_client_ = _rh->serviceClient<arm_navigation_msgs::GetMotionPlan>("ompl_planning/plan_kinematic_path");
    if(srv_client_.call(req,res)){
        if(res.trajectory.joint_trajectory.points.empty()){
            ROS_WARN_STREAM_NAMED(ARMS_LGRNM,"Planner returned with no trajectory. Returned with error code "
                                  <<res.error_code.val<<"("
                                  <<arm_navigation_msgs::armNavigationErrorCodeToString(res.error_code)<<")");
            return false;
        }
        else
            ROS_INFO_NAMED(ARMS_LGRNM,"Motion planning request returned with %d trajectory points.",res.trajectory.joint_trajectory.points.size());
    }
    else{
        ROS_ERROR_STREAM_NAMED(ARMS_LGRNM,"Call to ompl_planning/plan_kinematic_path service failed");
        return false;
    }
    ROS_INFO_NAMED(ARMS_LGRNM,"Motion plan request returned with solution");
    return true;
}

// from move_arm.cpp ros fuerte

bool TubeManipulation::Arms::_filter_trajectory(trajectory_msgs::JointTrajectory &trajectory_in,
                      trajectory_msgs::JointTrajectory &trajectory_out, arm_navigation_msgs::GetMotionPlan::Request mplan_req)
{
    if(trajectory_in.points.empty()){
        ROS_WARN_NAMED(ARMS_LGRNM,"No trajectory point");
        return false;
    }
    arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request req;
    arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response res;

    req.goal_constraints = mplan_req.motion_plan_request.goal_constraints;
    req.path_constraints = mplan_req.motion_plan_request.path_constraints;

    req.group_name =  mplan_req.motion_plan_request.group_name;

    req.allowed_time = ros::Duration(2);
    req.start_state.joint_state.name = trajectory_in.joint_names;
    req.start_state.joint_state.position.resize(trajectory_in.joint_names.size());

    trajectory_msgs::JointTrajectoryPoint traj_point = trajectory_in.points[0];
    req.start_state.joint_state.position = traj_point.positions;

    req.trajectory = trajectory_in;

    if(_traj_filter_client.call(req,res)){
        if(res.error_code.val == res.error_code.SUCCESS){
            trajectory_out = res.trajectory;
        }
        else{
            ROS_ERROR_STREAM_NAMED(ARMS_LGRNM,"Trajectory was not filtered. Error code: "<<res.error_code.val
                             <<"("<<arm_navigation_msgs::armNavigationErrorCodeToString(res.error_code)<<")");
            return false;
        }
    }
    else{
        ROS_ERROR_NAMED(ARMS_LGRNM,"Service call to filter_trajectory failed %s",_traj_filter_client.getService().c_str());
        return false;
    }
    return true;
}

TubeManipulation::CollisionCheck::CollisionCheck(ros::NodeHandlePtr nh,
                                                 collisionObjects::Ptr objPtr)
{
    _nh = nh;
    _mrkr_pub = _nh->advertise<visualization_msgs::MarkerArray>("tube_polishing/collision_check",100);
    _collision_models = new planning_environment::CollisionModels("robot_description");
    //ros::service::waitForService(GET_PLANNING_SCENE_NAME);
    //_set_scn_client = _nh->serviceClient<arm_navigation_msgs::GetPlanningScene>(GET_PLANNING_SCENE_NAME);

    _visualize = false;
    _good_color.a = _collision_color.a = _joint_limits_color.a = .8;
    _good_color.g = 1.0;
    _collision_color.r = 1.0;
    _joint_limits_color.b = 1.0;
    _point_markers.a = 1.0;
    _point_markers.r = 1.0;
    _point_markers.g = .8;
    _mrk_life_time = 0.2;

    _collision_obj_ptr = objPtr;

    /*_r_jnts.resize(7);
    _l_jnts.resize(7);
    _r_jnt_nms.resize(7);
    _l_jnt_nms.resize(7);*/

    _reset_state();

    _map[VALID] = "state is valid";
    _map[OUT_OF_BOUND_R] = "right joint(s) is/are out of bound";
    _map[IN_ENV_CLSN_R] = "right arm is in collision with environment";
    _map[IN_SLF_CLSN_R] = "right arm is in self collision";
    _map[OUT_OF_BOUND_L] = "left joint(s) is/are out of bound";
    _map[IN_ENV_CLSN_L] = "left arm is in collision with environment";
    _map[IN_SLF_CLSN_L] = "left arm is in self collision";
    _map[ERROR] = "unknown error";
}

/*void TubeManipulation::CollisionCheck::setCollisionObjectsPtr(collisionObjects::Ptr objPtr)
{
    //_att_obj_ptr.reset();
    _collision_obj_ptr = objPtr;
    refreshState();
}*/

/*void TubeManipulation::CollisionCheck::clearAttachedObj()
{
    arm_navigation_msgs::AttachedCollisionObject obj;
    _att_obj = obj;
    _att_obj.object.shapes.clear();
    refreshState();
}*/

void TubeManipulation::CollisionCheck::enableVisualization(){
    _visualize = true;
}
void TubeManipulation::CollisionCheck::disableVisualization(){
    _visualize = false;
}

TubeManipulation::CollisionCheck::~CollisionCheck(){
    _collision_models->revertPlanningScene(_state);
}

void TubeManipulation::CollisionCheck::refreshState(void){
    _collision_models->revertPlanningScene(_state);
    _reset_state();
}

void TubeManipulation::CollisionCheck::_reset_state(void)
{
    //arm_navigation_msgs::GetPlanningScene::Request scn_req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response scn_res;

//    if(!(_att_obj_ptr==NULL)){
//        if(!_att_obj_ptr->object.shapes.empty()){
//            scn_req.planning_scene_diff.attached_collision_objects.push_back(*_att_obj_ptr);
//            //_scn.attached_collision_objects.push_back(*_att_obj_ptr);
//        }
//    }
//    else{
//        ROS_WARN_NAMED(COLCHK_LGRNM,"Attached Object Pointer is NULL!");
//    }
 /*   if(!_set_scn_client.call(scn_req,scn_res)){
        ROS_ERROR_NAMED(COLCHK_LGRNM,"Can't set planning scene");
        return;
    }*/
    scn_res = _collision_obj_ptr->setPlanningScene();
    _scn = scn_res.planning_scene;
    _state = _collision_models->setPlanningScene(_scn);
    _r_jnt_nms = _collision_models->getKinematicModel()->getModelGroup("right_arm")->getJointModelNames();
    _l_jnt_nms = _collision_models->getKinematicModel()->getModelGroup("left_arm")->getJointModelNames();
    _r_lnk_nms = _collision_models->getKinematicModel()->getModelGroup("right_arm")->getUpdatedLinkModelNames();
    _l_lnk_nms = _collision_models->getKinematicModel()->getModelGroup("left_arm")->getUpdatedLinkModelNames();

    _state->getJointStateGroup("right_arm")->getKinematicStateValues(_r_jnt_values);
    _actual_r_jnts.resize(_r_jnt_nms.size());
    for(size_t i=0; i<_r_jnt_nms.size(); i++)
        _actual_r_jnts[i] = _r_jnt_values[_r_jnt_nms[i].c_str()];

    _actual_l_jnts.resize(_l_jnt_nms.size());
    _state->getJointStateGroup("left_arm")->getKinematicStateValues(_l_jnt_values);
    for(size_t i=0; i<_l_jnt_nms.size(); i++)
        _actual_l_jnts[i] = _l_jnt_values[_l_jnt_nms[i].c_str()];
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
    for(int i=0; i<_actual_r_jnts.size(); i++){
        std::cout<<i<<" > \t"<<_actual_r_jnts[i]<<"\t\t"<<_actual_l_jnts[i]<<std::endl;
    }
    std::cout<<std::endl;
}

bool TubeManipulation::CollisionCheck::isRightArmStateValid(std::vector<double> &joints){
    ROS_ASSERT_MSG(joints.size()==7,"TubeManipulation - Number of right joint values is not 7");
    _r_jnts = joints;
    _r_jnt_values.clear();

    for(size_t i=0; i<_r_jnts.size(); i++){
        _r_jnt_values[_r_jnt_nms[i]] = _r_jnts[i];
    }
    _mrkr_arr.markers.clear();
    bool res = _is_right_arm_state_valid();
    _mrkr_pub.publish(_mrkr_arr);
    return res;
}

bool TubeManipulation::CollisionCheck::isLeftArmStateValid(std::vector<double> &joints){
    ROS_ASSERT_MSG(joints.size()==7,"TubeManipulation - Number of right joint values is not 7");
    _l_jnts = joints;
    _l_jnt_values.clear();

    for(size_t i=0; i<_l_jnts.size(); i++){
        _l_jnt_values[_l_jnt_nms[i]] = _l_jnts[i];
    }
    _mrkr_arr.markers.clear();
    bool res = _is_left_arm_state_valid();
    _mrkr_pub.publish(_mrkr_arr);
    return res;
}

bool TubeManipulation::CollisionCheck::isStateValid(std::vector<double> &right_joints, std::vector<double> &left_joints)
{
    ROS_ASSERT_MSG(right_joints.size()==7,"TubeManipulation - Number of right joint values is not 7");
    ROS_ASSERT_MSG(left_joints.size()==7,"TubeManipulation - Number of left joint values is not 7");

    _r_jnts = right_joints;
    _l_jnts = left_joints;

    _r_jnt_values.clear();
    _l_jnt_values.clear();

    for(size_t i=0; i<_r_jnts.size(); i++){
        _r_jnt_values[_r_jnt_nms[i]] = _r_jnts[i];
        _l_jnt_values[_l_jnt_nms[i]] = _l_jnts[i];
    }
    _mrkr_arr.markers.clear();
    if(!_is_right_arm_state_valid()){
        _mrkr_pub.publish(_mrkr_arr);
        return false;
    }
    if(!_is_left_arm_state_valid()){
        _mrkr_pub.publish(_mrkr_arr);
        return false;
    }
    _mrkr_pub.publish(_mrkr_arr);
    return true;
}

bool TubeManipulation::CollisionCheck::_is_right_arm_state_valid()
{
    bool is_valid = true;

    std_msgs::ColorRGBA color;
    color = _good_color;

    _state->setKinematicState(_r_jnt_values);  //set test state for right arm
    if(!_state->areJointsWithinBounds(_r_jnt_nms)){
        ROS_DEBUG_NAMED(COLCHK_LGRNM,"Right joints are out of bound");
        color = _joint_limits_color;
        is_valid = false;
        _error = OUT_OF_BOUND_R;
    } else if(_collision_models->isKinematicStateInCollision(*_state)){
        ROS_DEBUG_NAMED(COLCHK_LGRNM,"Right arm state is in collision");
        color = _collision_color;
        is_valid = false;
        _error = ERROR;
        if(_collision_models->isKinematicStateInEnvironmentCollision(*_state))
            _error = IN_ENV_CLSN_R;
        if(_collision_models->isKinematicStateInSelfCollision(*_state))
            _error = IN_SLF_CLSN_R;
        if(_visualize){
            _collision_models->getAllCollisionPointMarkers(*_state, _mrkr_arr,
                                 _point_markers, ros::Duration(_mrk_life_time));
        }
    }
    _state->setKinematicState(_actual_r_jnts); //reset to what was initially

    if(_visualize){
        _collision_models->getRobotMarkersGivenState(*_state, _mrkr_arr,
                                                     color,"right_arm",ros::Duration(_mrk_life_time),&_r_lnk_nms);
        _collision_models->getAttachedCollisionObjectMarkers(*_state, _mrkr_arr,
                                                             "right_arm",color,ros::Duration(_mrk_life_time), false, &_r_lnk_nms);
    }
    if(is_valid)
        _error = VALID;
    return is_valid;
}

bool TubeManipulation::CollisionCheck::_is_left_arm_state_valid()
{
    bool is_valid = true;

    std_msgs::ColorRGBA color;
    color = _good_color;

    _state->setKinematicState(_l_jnt_values); //set left test state
    if(!_state->areJointsWithinBounds(_l_jnt_nms)){
        ROS_DEBUG_NAMED(COLCHK_LGRNM,"Left joints are out of bound");
        color = _joint_limits_color;
        is_valid = false;
        _error = OUT_OF_BOUND_L;
    } else if(_collision_models->isKinematicStateInCollision(*_state)){
        ROS_DEBUG_NAMED(COLCHK_LGRNM,"Left arm state is in collision");
        color = _collision_color;
        is_valid = false;
        _error = ERROR;
        if(_collision_models->isKinematicStateInEnvironmentCollision(*_state)){
            _error = IN_ENV_CLSN_L;
        }
        if(_collision_models->isKinematicStateInSelfCollision(*_state)){
            _error = IN_SLF_CLSN_L;
        }
        if(_visualize){
            _collision_models->getAllCollisionPointMarkers(*_state, _mrkr_arr,
                                 _point_markers, ros::Duration(_mrk_life_time));
        }
    }
    _state->setKinematicState(_actual_l_jnts); //reset to what was initially

    if(_visualize){
        _collision_models->getRobotMarkersGivenState(*_state, _mrkr_arr,
                                                     color,"left_arm",ros::Duration(_mrk_life_time),&_l_lnk_nms);
        _collision_models->getAttachedCollisionObjectMarkers(*_state, _mrkr_arr,
                                                             "left_arm",color,ros::Duration(_mrk_life_time), false, &_l_lnk_nms);
    }
    if(is_valid)
        _error = VALID;
    return is_valid;
}

bool TubeManipulation::CollisionCheck::_is_state_valid()
{
    bool is_valid = true;

    std_msgs::ColorRGBA color;
    color = _good_color;

    _state->setKinematicState(_r_jnt_values);  //set test state for right arm
    if(!_state->areJointsWithinBounds(_r_jnt_nms)){
        ROS_DEBUG_NAMED(COLCHK_LGRNM,"Right joints are out of bound");
        color = _joint_limits_color;
        is_valid = false;
        _error = OUT_OF_BOUND_R;
    } else if(_collision_models->isKinematicStateInCollision(*_state)){
        ROS_DEBUG_NAMED(COLCHK_LGRNM,"Right arm state is in collision");
        color = _collision_color;
        is_valid = false;
        _error = ERROR;
        if(_collision_models->isKinematicStateInEnvironmentCollision(*_state))
            _error = IN_ENV_CLSN_R;
        if(_collision_models->isKinematicStateInSelfCollision(*_state))
            _error = IN_SLF_CLSN_R;
        _collision_models->getAllCollisionPointMarkers(*_state, _mrkr_arr,
                                 _point_markers, ros::Duration(_mrk_life_time));
    }
    _state->setKinematicState(_actual_r_jnts); //reset to what was initially

    _state->setKinematicState(_l_jnt_values); //set left test state
    if(!_state->areJointsWithinBounds(_l_jnt_nms)){
        ROS_DEBUG_NAMED(COLCHK_LGRNM,"Left joints are out of bound");
        color = _joint_limits_color;
        is_valid = false;
        _error = OUT_OF_BOUND_L;
    } else if(_collision_models->isKinematicStateInCollision(*_state)){
        ROS_DEBUG_NAMED(COLCHK_LGRNM,"Left arm state is in collision");
        color = _collision_color;
        is_valid = false;
        _error = ERROR;
        if(_collision_models->isKinematicStateInEnvironmentCollision(*_state)){
            _error = IN_ENV_CLSN_L;
        }
        if(_collision_models->isKinematicStateInSelfCollision(*_state)){
            _error = IN_SLF_CLSN_L;
        }
        _collision_models->getAllCollisionPointMarkers(*_state, _mrkr_arr,
                                 _point_markers, ros::Duration(_mrk_life_time));
    }
    _state->setKinematicState(_actual_l_jnts); //reset to what was initially

    if(_visualize){
        _collision_models->getRobotMarkersGivenState(*_state, _mrkr_arr,
                                                     color,"right_arm",ros::Duration(_mrk_life_time),&_r_lnk_nms);
        _collision_models->getRobotMarkersGivenState(*_state, _mrkr_arr,
                                                     color,"left_arm",ros::Duration(_mrk_life_time),&_l_lnk_nms);
        _collision_models->getAttachedCollisionObjectMarkers(*_state, _mrkr_arr,
                                                             "right_arm",color,ros::Duration(_mrk_life_time), false, &_r_lnk_nms);
        _collision_models->getAttachedCollisionObjectMarkers(*_state, _mrkr_arr,
                                                             "left_arm",color,ros::Duration(_mrk_life_time), false, &_l_lnk_nms);
        _mrkr_pub.publish(_mrkr_arr);
        _mrkr_arr.markers.clear();
    }
    if(is_valid)
        _error = VALID;
    return is_valid;
}

void TubeManipulation::CollisionCheck::setMarkerLifeTime(double time){
    _mrk_life_time = time;
}

int TubeManipulation::CollisionCheck::getLastError(){
    return _error;
}

std::string TubeManipulation::CollisionCheck::getLastErrorAsString(){
    std::string str = _map[_error];
    return str;
}
