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
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <arm_navigation_msgs/utils.h>
#include <planning_environment/models/collision_models.h>
#include <vector>

#include "collisionObjects.h"
#include "utility.h"

#define MAX_JOINT_VEL 0.2

/*! \brief  Simple action server client definition for JointTrajectoryAction */
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;
//typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;

namespace TubeManipulation
{

/*class Trajectory
{
public:
    Trajectory();
    void setTrajectory(std::vector<double> &rightTraj, std::vector<double> &leftTraj);

private:
    trajectory_msgs::JointTrajectory _right_traj;
    trajectory_msgs::JointTrajectory _left_traj;
    std::vector<std::string> _r_jnt_nms;
    std::vector<std::string> _l_jnt_nms;
};*/


class CollisionCheck
{
public:
    CollisionCheck(ros::NodeHandlePtr nh, collisionObjects::Ptr objPtr);
    ~CollisionCheck();
    void refreshState(void);
    //void setAttachedObjPtr(arm_navigation_msgs::AttachedCollisionObject::Ptr attObjPtr);
    //void clearAttachedObj();
    void printState(void);
    bool isStateValid(std::vector<double> &right_joints,
                      std::vector<double> &left_joints);
    bool isRightArmStateValid(std::vector<double> &joints);
    bool isLeftArmStateValid(std::vector<double> &joints);
    void enableVisualization();
    void disableVisualization();
    void setMarkerLifeTime(double time);
    int  getLastError();
    std::string getLastErrorAsString();
    typedef boost::shared_ptr<TubeManipulation::CollisionCheck> Ptr;
    enum{
        VALID,
        OUT_OF_BOUND_R,
        IN_ENV_CLSN_R,
        IN_SLF_CLSN_R,
        OUT_OF_BOUND_L,
        IN_ENV_CLSN_L,
        IN_SLF_CLSN_L,
        ERROR
    };

private:
    ros::NodeHandlePtr _nh;
    ros::ServiceClient _set_scn_client; /*!< set planning scene diff */
    arm_navigation_msgs::PlanningScene _scn;
    planning_environment::CollisionModels *_collision_models;
    collisionObjects::Ptr _collision_obj_ptr;
    std::map<std::string, double> _r_jnt_values;
    std::map<std::string, double> _l_jnt_values;
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
    double _mrk_life_time;
    planning_models::KinematicState* _state;
    bool _visualize;
    int _error;

    std_msgs::ColorRGBA _good_color;
    std_msgs::ColorRGBA _collision_color;
    std_msgs::ColorRGBA _joint_limits_color;
    std_msgs::ColorRGBA _point_markers;
    std::map<int, std::string> _map;

    bool _is_state_valid();
    bool _is_right_arm_state_valid();
    bool _is_left_arm_state_valid();
    void _reset_state();
};

/*! \brief Class to move both hands in sync for graspped object by both hands. 
 *
 *  This class takes in offset in transforms for both hands and _obj_pose_traj as pose array.
 *  It generates joint trajectory for both hands using GetconstraintAwarePositionIK service 
 *  and executes both trajectory in synchronized fashion.
 */
class Arms /*: public Trajectory*/
{
public:
    Arms(ros::NodeHandlePtr nh, collisionObjects::Ptr colObjPtr);
    ~Arms();
    bool genTrajectory(geometry_msgs::PoseArray &objPoseArray,
                       geometry_msgs::Pose &rightArmOffset,
                       geometry_msgs::Pose &leftArmOffset,
                       std::vector<double> &rightJointTraj,
                       std::vector<double> &leftJointTraj);
    bool genLeftTrajectory(std::vector<double> &jointTrajectory,
                           geometry_msgs::Pose &leftArmOffset);
    bool genRightTrajectory(std::vector<double> &jointTrajectory,
                            geometry_msgs::Pose &rightArmOffset);
    //bool executeJointTrajectory();
    bool executeJointTrajectoryWithSync(std::vector<double> &qRight,
                                std::vector<double> &qLeft);
    bool moveRightArm(geometry_msgs::Pose pose);
    bool simpleMoveRightArm(geometry_msgs::Pose pose);
    bool moveLeftArm(geometry_msgs::Pose pose);
    bool simpleMoveLeftArm(geometry_msgs::Pose pose);
    bool getRightArmIK(geometry_msgs::Pose pose,
                       sensor_msgs::JointState &jointState);
    bool getRightArmIK(geometry_msgs::Pose pose,
                       std::vector<double> &jointsOut);
    bool getLeftArmIK(geometry_msgs::Pose pose,
                      sensor_msgs::JointState &jointState);
    bool getLeftArmIK(geometry_msgs::Pose pose,
                       std::vector<double> &jointsOut);
    bool getSimpleRightArmIK(geometry_msgs::Pose pose,
                                 sensor_msgs::JointState &jointState);
    bool getSimpleRightArmIK(geometry_msgs::Pose pose,
                                 std::vector<double> &jointsOut);
    bool getSimpleLeftArmIK(geometry_msgs::Pose pose,
                                 sensor_msgs::JointState &jointState);
    bool getSimpleLeftArmIK(geometry_msgs::Pose pose,
                                 std::vector<double> &jointsOut);
    bool getRegraspPoseRight(arm_navigation_msgs::AttachedCollisionObject::Ptr attObjPtr,
                             geometry_msgs::Pose crnt_grasp,
                             geometry_msgs::Pose wrist_pose,
                             geometry_msgs::Pose other_hand_grasp,
                             geometry_msgs::Pose &obj_pose_out,
                             std::vector<double> &ik_soln);
    bool getRegraspPoseLeft(arm_navigation_msgs::AttachedCollisionObject::Ptr attObjPtr,
                            geometry_msgs::Pose crnt_grasp,
                             geometry_msgs::Pose wrist_pose,
                             geometry_msgs::Pose other_hand_grasp,
                             geometry_msgs::Pose &obj_pose_out,
                             std::vector<double> &ik_soln);
    bool moveRightArmWithMPlanning(std::vector<double> &ik_soln);
    bool moveRightArmWithMPlanning(geometry_msgs::Pose pose);
    bool moveLeftArmWithMPlanning(std::vector<double> &ik_soln);
    bool moveLeftArmWithMPlanning(geometry_msgs::Pose pose);
    geometry_msgs::Pose getRightArmFK();
    geometry_msgs::Pose getRightArmFK(std::vector<double> &left_joints);
    geometry_msgs::Pose getLeftArmFK();
    geometry_msgs::Pose getLeftArmFK(std::vector<double> &left_joints);
    void getRightJoints(std::vector<double> &joints);
    void getLeftJoints(std::vector<double> &joints);
    bool moveRightArmToCloudCapturePose(const tf::Vector3 &sensorOrig,
                                        const tf::Vector3 &startOrig,
                                        const double minSensorDistance,
                                        std::vector<double> &ikSoln);

    typedef boost::shared_ptr<TubeManipulation::Arms> Ptr;

private:
    ros::NodeHandlePtr _rh;
    TrajClient* _traj_client_r; /*!< Right arm trajectory action client. */
    TrajClient* _traj_client_l; /*!< Left arm trajectory action client. */
    //ros::ServiceClient _set_pln_scn_client; /*!< get planning scene diff */
    ros::ServiceClient _ik_client_r; /*!< Right arm IK client. */
    ros::ServiceClient _ik_client_l; /*!< Left arm IK client. */
    ros::ServiceClient _fk_client_r;
    ros::ServiceClient _fk_client_l;
    ros::ServiceClient _smpl_ik_client_r; /*!< Left arm IK client. */
    ros::ServiceClient _smpl_ik_client_l; /*!< Left arm IK client. */
    ros::ServiceClient _query_client_r; /*!< Right arm kinematic solver info query client. */
    ros::ServiceClient _query_client_l; /*!< Left arm kinematic solver info query client. */
    ros::ServiceClient _traj_unnormalizer_client; /*!< Joint trajectory unnormalizer filter client. */
    ros::ServiceClient _traj_filter_client;
    ros::Publisher _att_obj_pub;
    ros::Publisher _test_pose_pub;
    pr2_controllers_msgs::JointTrajectoryGoal _right_goal, _left_goal; /*!< Joint trajectory goal to execute joint trajectory. */
    std::vector<double> _right_joint_traj,_left_joint_traj; /*!< Double linear array to store joint trajectory. */
    tf::Transform _left_wrist_offset; /*!< Offset of left arm wrist_roll link from object. */
    tf::Transform _right_wrist_offset; /*!< Offset of right arm wrist_roll link from object. */
    geometry_msgs::PoseArray _obj_pose_traj; /*!< Pose trajectory of an object. */
    std::vector<std::string> _r_jnt_nms;
    std::vector<std::string> _l_jnt_nms;
    CollisionCheck::Ptr _collision_check;
    std::map<std::string, std::pair<double, double> > _joint_bounds;
    //arm_navigation_msgs::AttachedCollisionObject::Ptr _att_obj_ptr;
    collisionObjects::Ptr _collision_objects;

    void _get_right_joints(std::vector<double> &joint_state);
    void _get_left_joints(std::vector<double> &joint_state);

    void _get_joint_trajectory_goal(const std::vector<double> &qIn,
                                    pr2_controllers_msgs::JointTrajectoryGoal &goal,
                                    const std::vector<std::string> &jnt_nms);
    void _call_joints_unnormalizer(pr2_controllers_msgs::JointTrajectoryGoal &traj_goal);
    void _sync_start_times(pr2_controllers_msgs::JointTrajectoryGoal &right_goal, pr2_controllers_msgs::JointTrajectoryGoal &left_goal);

    //bool _call_right_arm_gpik(std::vector<double> &right_joint_trajectory);
    //bool _call_left_arm_gpik(std::vector<double> &left_joint_trajectory);
    void _call_right_joints_unnormalizer(void);
    void _call_left_joints_unnormalizer(void);

    bool _get_right_arm_ik(geometry_msgs::Pose pose,
                          sensor_msgs::JointState &joint_state,
                          std::vector<double> &seed_state);
    bool _get_right_arm_ik(geometry_msgs::Pose &pose,
                          std::vector<double> &joints,
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
    bool _get_left_arm_ik(geometry_msgs::Pose &pose,
                          std::vector<double> &joints,
                          std::vector<double> &seed_state);
    bool _get_simple_left_arm_ik(geometry_msgs::Pose &pose,
                          sensor_msgs::JointState &joint_state,
                          std::vector<double> &seed_state);
    bool _get_simple_left_arm_ik(geometry_msgs::Pose &pose,
                                  std::vector<double> &joints,
                                  std::vector<double> &seed_state);
    bool _gen_trajectory(std::vector<double> &right_joint_traj,
                           std::vector<double> &left_joint_traj);
    bool _move_right_arm_with_mplning(std::vector<double> &ik_joints);
    bool _move_left_arm_with_mplning(std::vector<double> &ik_joints);
    bool _get_motion_plan(arm_navigation_msgs::GetMotionPlan::Request &req,
                          arm_navigation_msgs::GetMotionPlan::Response &res);
    bool _filter_trajectory(trajectory_msgs::JointTrajectory &trajectory_in,
                            trajectory_msgs::JointTrajectory &trajectory_out,
                            arm_navigation_msgs::GetMotionPlan::Request mplan_req);
    void _fill_trajectory_msg(trajectory_msgs::JointTrajectory &trajectory_in,
                              trajectory_msgs::JointTrajectory &trajectory_out);
    void _execute_joint_trajectory(trajectory_msgs::JointTrajectory &right_traj,
                                   trajectory_msgs::JointTrajectory &left_traj);
    bool _handle_planning_error(arm_navigation_msgs::GetMotionPlan::Request &req,
                                arm_navigation_msgs::GetMotionPlan::Response &res);
    void _get_bounds_from_description();
    bool _start_is_goal(arm_navigation_msgs::GetMotionPlan::Request &req,
                        arm_navigation_msgs::GetMotionPlan::Response &res);
    void _publish_pose(const geometry_msgs::Pose &pose);

    geometry_msgs::Pose _get_right_fk(std::vector<double> &joints);
    geometry_msgs::Pose _get_left_fk(std::vector<double> &joints);
};//Arms

}//TubeManipulation

#endif // TUBEMANIPULATION_H
