#ifndef TUBEMANIPULATION_H
#define TUBEMANIPULATION_H
#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <arm_navigation_msgs/FilterJointTrajectory.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/RobotState.h> //don't need it.
#include <planning_environment/models/collision_models.h>
#include <vector>

#include "utility.h"

#define MAX_JOINT_VEL 0.2
#define SET_PLANNING_SCENE_DIFF_NAME "/environment_server/set_planning_scene_diff"

/*! \brief  Simple action server client definition for JointTrajectoryAction */
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

namespace TubeManipulation
{

/*! \brief Class to move both hands in sync for graspped object by both hands. 
 *
 *  This class takes in offset in transforms for both hands and _obj_pose_traj as pose array.
 *  It generates joint trajectory for both hands using GetconstraintAwarePositionIK service 
 *  and executes both trajectory in synchronized fashion.
 */
class Arms
{
public:
    Arms(ros::NodeHandlePtr rh);
    bool genTrajectory();
    bool genTrajectory(std::vector<double> &rightJointTraj, std::vector<double> &lefttJointTraj);
    bool genLeftTrajectory(std::vector<double> &jointTrajectory);  //Prerequisites: leftWristOffset and objPoseTraj
    bool genRightTrajectory(std::vector<double> &jointTrajectory); //Prerequisites: rightWristOffset and objPoseTraj
    bool executeJointTrajectory();
    bool executeJointTrajectory(std::vector<double> &qRight,
                                std::vector<double> &qLeft);
    bool moveRightArm(geometry_msgs::Pose pose);
    bool simpleMoveRightArm(geometry_msgs::Pose pose);
    bool moveLeftArm(geometry_msgs::Pose pose);
    bool simpleMoveLeftArm(geometry_msgs::Pose pose);
    bool getRightArmIK(geometry_msgs::Pose pose,
                       sensor_msgs::JointState &jointState);
    bool getLeftArmIK(geometry_msgs::Pose pose,
                      sensor_msgs::JointState &jointState);
    bool getSimpleRightArmIK(geometry_msgs::Pose pose,
                                 sensor_msgs::JointState &jointState);
    bool getSimpleRightArmIK(geometry_msgs::Pose pose,
                                 std::vector<double> &jointState);
    bool getSimpleLeftArmIK(geometry_msgs::Pose pose,
                                 sensor_msgs::JointState &jointState);
    void setObjPoseTrajectory(geometry_msgs::PoseArray &pose_array);
    void setWristOffset(tf::Transform &right_offset, tf::Transform &left_offset);
    void setWristOffset(geometry_msgs::Pose &right_offset, geometry_msgs::Pose &left_offset);
    bool _get_regrasp_pose_right(geometry_msgs::Pose crnt_grasp,
                                 geometry_msgs::Pose wrist_pose,
                                 geometry_msgs::Pose right_grasp,
                                 geometry_msgs::Pose left_grasp,
                                 arm_navigation_msgs::AttachedCollisionObject att_obj,
                                 geometry_msgs::Pose &obj_pose_out);

private:
    ros::NodeHandlePtr _rh;
    TrajClient* _traj_client_r; /*!< Right arm trajectory action client. */
    TrajClient* _traj_client_l; /*!< Left arm trajectory action client. */
    ros::ServiceClient _set_pln_scn_client; /*!< set planning scene diff */
    ros::ServiceClient _get_pln_scn_client; /*!< get planning scene diff */
    ros::ServiceClient _ik_client_r; /*!< Right arm IK client. */
    ros::ServiceClient _ik_client_l; /*!< Left arm IK client. */
    ros::ServiceClient _fk_client_r;
    ros::ServiceClient _fk_client_l;
    ros::ServiceClient _smpl_ik_client_r; /*!< Left arm IK client. */
    ros::ServiceClient _smpl_ik_client_l; /*!< Left arm IK client. */
    ros::ServiceClient _query_client_r; /*!< Right arm kinematic solver info query client. */
    ros::ServiceClient _query_client_l; /*!< Left arm kinematic solver info query client. */
    ros::ServiceClient _filter_trajectory_client; /*!< Joint trajectory unnormalizer filter client. */
    pr2_controllers_msgs::JointTrajectoryGoal _right_goal, _left_goal; /*!< Joint trajectory goal to execute joint trajectory. */
    std::vector<double> _right_joint_traj,_left_joint_traj; /*!< Double linear array to store joint trajectory. */
    tf::Transform _left_wrist_offset; /*!< Offset of left arm wrist_roll link from object. */
    tf::Transform _right_wrist_offset; /*!< Offset of right arm wrist_roll link from object. */
    geometry_msgs::PoseArray _obj_pose_traj; /*!< Pose trajectory of an object. */

    void _get_right_goal();
    void _get_left_goal();
    void _sync_start_times(void);
    //bool _call_right_arm_gpik(std::vector<double> &right_joint_trajectory);
    //bool _call_left_arm_gpik(std::vector<double> &left_joint_trajectory);
    void _call_right_joints_unnormalizer(void);
    void _call_left_joints_unnormalizer(void);
    void _get_right_joints(std::vector<double> &joint_state);
    void _get_left_joints(std::vector<double> &joint_state);
    void _get_default_right_joints(std::vector<double> &joint_state);
    void _get_default_left_joints(std::vector<double> &joint_state);
    bool _get_right_arm_ik(geometry_msgs::Pose pose,
                          sensor_msgs::JointState &joint_state,
                          std::vector<double> &seed_state);
    bool _get_simple_right_arm_ik(geometry_msgs::Pose &pose,
                          sensor_msgs::JointState &joint_state,
                          std::vector<double> &seed_state);
    bool _get_simple_right_arm_ik(geometry_msgs::Pose &pose,
                                  std::vector<double> &joints,
                                  std::vector<double> &seed_state);
    bool _get_left_arm_ik(geometry_msgs::Pose pose,
                          sensor_msgs::JointState &joint_state,
                          std::vector<double> &seed_state);
    bool _get_simple_left_arm_ik(geometry_msgs::Pose &pose,
                          sensor_msgs::JointState &joint_state,
                          std::vector<double> &seed_state);
    bool _get_simple_left_arm_ik(geometry_msgs::Pose &pose,
                                  std::vector<double> &joints,
                                  std::vector<double> &seed_state);
    bool _gen_trarajectory(std::vector<double> &right_joint_traj,
                           std::vector<double> &left_joint_traj);

    geometry_msgs::Pose _get_right_fk(std::vector<double> &joints);
    geometry_msgs::Pose _get_left_fk(std::vector<double> &joints);

};

class CollisionCheck
{
public:
    CollisionCheck(ros::NodeHandlePtr nh);
    ~CollisionCheck();
    void refreshState(void);
    void setAttachedObj(arm_navigation_msgs::AttachedCollisionObject &attachedObj);
    void clearAttachedObj(void);
    void printState(void);
    bool isStateValid(std::vector<double> &right_joints,
                      std::vector<double> &left_joints);
    void enableVisualization();
    void disableVisualization();
private:
    ros::NodeHandlePtr _nh;
    ros::ServiceClient _get_scn_client; /*!< get planning scene diff */
    arm_navigation_msgs::PlanningScene _scn;
    planning_environment::CollisionModels *_collision_models;
    arm_navigation_msgs::AttachedCollisionObject _att_obj;
    arm_navigation_msgs::GetPlanningScene::Request _scn_req;
    arm_navigation_msgs::GetPlanningScene::Response _scn_res;
    std::map<std::string, double> _jnt_values;
    std::vector<double> _r_jnts;
    std::vector<double> _l_jnts;
    std::vector<std::string> _r_jnt_nms;
    std::vector<std::string> _r_lnk_nms;
    std::vector<std::string> _l_jnt_nms;
    std::vector<std::string> _l_lnk_nms;
    std::vector<double> _actual_r_jnts;
    std::vector<double> _actual_l_jnts;
    ros::Publisher _mrkr_pub;
    visualization_msgs::MarkerArray _mrkr_arr;
    planning_models::KinematicState* _state;
    bool _visualize;

    std_msgs::ColorRGBA _good_color;
    std_msgs::ColorRGBA _collision_color;
    std_msgs::ColorRGBA _joint_limits_color;
    std_msgs::ColorRGBA _point_markers;

    bool _is_state_valid();
    void _reset_state();
};

}//TubeManipulation

#endif // TUBEMANIPULATION_H
