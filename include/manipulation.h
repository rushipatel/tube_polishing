#ifndef MANIPULATION_H
#define MANIPULATION_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <manipulation_msgs/Grasp.h>

#include <geometry_msgs/Pose.h>
/*#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/tf.h>*/

#include "utility.h"



class manipulation
{
public:
    manipulation(ros::NodeHandlePtr nh);
    bool moveRightArm(geometry_msgs::Pose pose);
    void printState();
    void experimental();
    typedef boost::shared_ptr<manipulation> Ptr;

private:
    ros::NodeHandlePtr _nh;
    boost::shared_ptr<moveit::planning_interface::MoveGroup> _right_group;
    boost::shared_ptr<moveit::planning_interface::MoveGroup> _left_group;
    ros::Publisher _display_publisher;
    moveit_msgs::DisplayTrajectory _display_traj;

    void _visialize_plan(moveit::planning_interface::MoveGroup::Plan plan);
};




/******************************************************************************/
/******************************************************************************/
/***SOME OLD CORE FOR REFERENCE****/


//#define MAX_JOINT_VEL 0.2
//#define SET_PLANNING_SCENE_DIFF_NAME "/environment_server/set_planning_scene_diff"

///*! \brief  Simple action server client definition for JointTrajectoryAction */
//typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;
//typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;

//namespace TubeManipulation
//{


//class CollisionCheck
//{
//public:
//    CollisionCheck(ros::NodeHandlePtr nh);
//    ~CollisionCheck();
//    void refreshState(void);
//    void setAttachedObj(arm_navigation_msgs::AttachedCollisionObject &attachedObj);
//    void clearAttachedObj(void);
//    void printState(void);
//    bool isStateValid(std::vector<double> &right_joints,
//                      std::vector<double> &left_joints);
//    void enableVisualization();
//    void disableVisualization();
//    void setMarkerLifeTime(double time);
//    typedef boost::shared_ptr<TubeManipulation::CollisionCheck> Ptr;

//private:
//    ros::NodeHandlePtr _nh;
//    ros::ServiceClient _get_scn_client; /*!< get planning scene diff */
//    arm_navigation_msgs::PlanningScene _scn;
//    planning_environment::CollisionModels *_collision_models;
//    arm_navigation_msgs::AttachedCollisionObject _att_obj;
//    arm_navigation_msgs::GetPlanningScene::Request _scn_req;
//    arm_navigation_msgs::GetPlanningScene::Response _scn_res;
//    std::map<std::string, double> _jnt_values;
//    std::vector<double> _r_jnts;
//    std::vector<double> _l_jnts;
//    std::vector<std::string> _r_jnt_nms;
//    std::vector<std::string> _r_lnk_nms;
//    std::vector<std::string> _l_jnt_nms;
//    std::vector<std::string> _l_lnk_nms;
//    std::vector<double> _actual_r_jnts;
//    std::vector<double> _actual_l_jnts;
//    ros::Publisher _mrkr_pub;
//    visualization_msgs::MarkerArray _mrkr_arr;
//    double _mrk_life_time;
//    planning_models::KinematicState* _state;
//    bool _visualize;

//    std_msgs::ColorRGBA _good_color;
//    std_msgs::ColorRGBA _collision_color;
//    std_msgs::ColorRGBA _joint_limits_color;
//    std_msgs::ColorRGBA _point_markers;

//    bool _is_state_valid();
//    void _reset_state();
//};

///*! \brief Class to move both hands in sync for graspped object by both hands.
// *
// *  This class takes in offset in transforms for both hands and _obj_pose_traj as pose array.
// *  It generates joint trajectory for both hands using GetconstraintAwarePositionIK service
// *  and executes both trajectory in synchronized fashion.
// */
//class Arms
//{
//public:
//    Arms(ros::NodeHandlePtr rh);
//    bool genTrajectory();
//    bool genTrajectory(std::vector<double> &rightJointTraj, std::vector<double> &leftJointTraj);
//    bool genLeftTrajectory(std::vector<double> &jointTrajectory);  //Prerequisites: leftWristOffset and objPoseTraj
//    bool genRightTrajectory(std::vector<double> &jointTrajectory); //Prerequisites: rightWristOffset and objPoseTraj
//    bool executeJointTrajectory();
//    bool executeJointTrajectory(std::vector<double> &qRight,
//                                std::vector<double> &qLeft);
//    bool moveRightArm(geometry_msgs::Pose pose);
//    bool simpleMoveRightArm(geometry_msgs::Pose pose);
//    bool moveLeftArm(geometry_msgs::Pose pose);
//    bool simpleMoveLeftArm(geometry_msgs::Pose pose);
//    bool getRightArmIK(geometry_msgs::Pose pose,
//                       sensor_msgs::JointState &jointState);
//    bool getRightArmIK(geometry_msgs::Pose pose,
//                       std::vector<double> &jointsOut);
//    bool getLeftArmIK(geometry_msgs::Pose pose,
//                      sensor_msgs::JointState &jointState);
//    bool getLeftArmIK(geometry_msgs::Pose pose,
//                       std::vector<double> &jointsOut);
//    bool getSimpleRightArmIK(geometry_msgs::Pose pose,
//                                 sensor_msgs::JointState &jointState);
//    bool getSimpleRightArmIK(geometry_msgs::Pose pose,
//                                 std::vector<double> &jointsOut);
//    bool getSimpleLeftArmIK(geometry_msgs::Pose pose,
//                                 sensor_msgs::JointState &jointState);
//    void setObjPoseTrajectory(geometry_msgs::PoseArray &pose_array);
//    void setWristOffset(tf::Transform &right_offset, tf::Transform &left_offset);
//    void setWristOffset(geometry_msgs::Pose &right_offset, geometry_msgs::Pose &left_offset);
//    bool getRegraspPoseRight(geometry_msgs::Pose crnt_grasp,
//                                 geometry_msgs::Pose wrist_pose,
//                                 geometry_msgs::Pose other_hand_grasp,
//                                 arm_navigation_msgs::AttachedCollisionObject att_obj,
//                                 geometry_msgs::Pose &obj_pose_out);
//    bool getRegraspPoseLeft(geometry_msgs::Pose crnt_grasp,
//                                 geometry_msgs::Pose wrist_pose,
//                                 geometry_msgs::Pose other_hand_grasp,
//                                 arm_navigation_msgs::AttachedCollisionObject att_obj,
//                                 geometry_msgs::Pose &obj_pose_out);
//    bool moveRightArmWithMPlanning(geometry_msgs::Pose pose);
//    bool moveRightArmWithMPlanning(arm_navigation_msgs::AttachedCollisionObject &attObj,
//                                   geometry_msgs::Pose pose);
//    bool moveLeftArmWithMPlanning(geometry_msgs::Pose pose);
//    bool moveLeftArmWithMPlanning(arm_navigation_msgs::AttachedCollisionObject &attObj, geometry_msgs::Pose pose);
//    geometry_msgs::Pose getRightArmFK();
//    geometry_msgs::Pose getRightArmFK(std::vector<double> &left_joints);
//    geometry_msgs::Pose getLeftArmFK();
//    geometry_msgs::Pose getLeftArmFK(std::vector<double> &left_joints);

//    typedef boost::shared_ptr<TubeManipulation::Arms> Ptr;

//private:
//    ros::NodeHandlePtr _rh;
//    TrajClient* _traj_client_r; /*!< Right arm trajectory action client. */
//    TrajClient* _traj_client_l; /*!< Left arm trajectory action client. */
//    MoveArmClient* _mv_arm_client_r;
//    MoveArmClient* _mv_arm_client_l;
//    ros::ServiceClient _set_pln_scn_client; /*!< set planning scene diff */
//    ros::ServiceClient _get_pln_scn_client; /*!< get planning scene diff */
//    ros::ServiceClient _ik_client_r; /*!< Right arm IK client. */
//    ros::ServiceClient _ik_client_l; /*!< Left arm IK client. */
//    ros::ServiceClient _fk_client_r;
//    ros::ServiceClient _fk_client_l;
//    ros::ServiceClient _smpl_ik_client_r; /*!< Left arm IK client. */
//    ros::ServiceClient _smpl_ik_client_l; /*!< Left arm IK client. */
//    ros::ServiceClient _query_client_r; /*!< Right arm kinematic solver info query client. */
//    ros::ServiceClient _query_client_l; /*!< Left arm kinematic solver info query client. */
//    ros::ServiceClient _traj_unnormalizer_client; /*!< Joint trajectory unnormalizer filter client. */
//    ros::ServiceClient _traj_filter_client;
//    ros::Publisher _att_obj_pub;
//    pr2_controllers_msgs::JointTrajectoryGoal _right_goal, _left_goal; /*!< Joint trajectory goal to execute joint trajectory. */
//    std::vector<double> _right_joint_traj,_left_joint_traj; /*!< Double linear array to store joint trajectory. */
//    tf::Transform _left_wrist_offset; /*!< Offset of left arm wrist_roll link from object. */
//    tf::Transform _right_wrist_offset; /*!< Offset of right arm wrist_roll link from object. */
//    geometry_msgs::PoseArray _obj_pose_traj; /*!< Pose trajectory of an object. */
//    std::vector<std::string> _r_jnt_nms;
//    std::vector<std::string> _l_jnt_nms;
//    CollisionCheck::Ptr _collision_check;

//    void _get_right_goal();
//    void _get_left_goal();
//    void _sync_start_times(void);
//    //bool _call_right_arm_gpik(std::vector<double> &right_joint_trajectory);
//    //bool _call_left_arm_gpik(std::vector<double> &left_joint_trajectory);
//    void _call_right_joints_unnormalizer(void);
//    void _call_left_joints_unnormalizer(void);
//    void _get_right_joints(std::vector<double> &joint_state);
//    void _get_left_joints(std::vector<double> &joint_state);
//    void _get_default_right_joints(std::vector<double> &joint_state);
//    void _get_default_left_joints(std::vector<double> &joint_state);
//    bool _get_right_arm_ik(geometry_msgs::Pose pose,
//                          sensor_msgs::JointState &joint_state,
//                          std::vector<double> &seed_state);
//    bool _get_right_arm_ik(geometry_msgs::Pose &pose,
//                          std::vector<double> &joints,
//                          std::vector<double> &seed_state);
//    bool _get_simple_right_arm_ik(geometry_msgs::Pose &pose,
//                          sensor_msgs::JointState &joint_state,
//                          std::vector<double> &seed_state);
//    bool _get_simple_right_arm_ik(geometry_msgs::Pose &pose,
//                                  std::vector<double> &joints,
//                                  std::vector<double> &seed_state);
//    bool _get_left_arm_ik(geometry_msgs::Pose pose,
//                          sensor_msgs::JointState &joint_state,
//                          std::vector<double> &seed_state);
//    bool _get_left_arm_ik(geometry_msgs::Pose &pose,
//                          std::vector<double> &joints,
//                          std::vector<double> &seed_state);
//    bool _get_simple_left_arm_ik(geometry_msgs::Pose &pose,
//                          sensor_msgs::JointState &joint_state,
//                          std::vector<double> &seed_state);
//    bool _get_simple_left_arm_ik(geometry_msgs::Pose &pose,
//                                  std::vector<double> &joints,
//                                  std::vector<double> &seed_state);
//    bool _gen_trarajectory(std::vector<double> &right_joint_traj,
//                           std::vector<double> &left_joint_traj);
//    bool _move_right_arm_with_mplning(arm_navigation_msgs::AttachedCollisionObject &attObj, geometry_msgs::Pose pose);
//    bool _move_left_arm_with_mplning(arm_navigation_msgs::AttachedCollisionObject &attObj, geometry_msgs::Pose pose);
//    bool _get_motion_plan(arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res);
//    bool _filter_trajectory(trajectory_msgs::JointTrajectory &trajectory_in,
//                            trajectory_msgs::JointTrajectory &trajectory_out, arm_navigation_msgs::GetMotionPlan::Request mplan_req);
//    void _fill_trajectory_msg(trajectory_msgs::JointTrajectory &trajectory_in,
//                              trajectory_msgs::JointTrajectory &trajectory_out);
//    void _execute_joint_trajectory(trajectory_msgs::JointTrajectory &right_traj, trajectory_msgs::JointTrajectory &left_traj);

//    geometry_msgs::Pose _get_right_fk(std::vector<double> &joints);
//    geometry_msgs::Pose _get_left_fk(std::vector<double> &joints);

//};

//}//manipulation

#endif // MANIPULATION_H
