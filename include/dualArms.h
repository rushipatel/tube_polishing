#ifndef DUALARMS_H
#define DUALARMS_H
#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <arm_navigation_msgs/FilterJointTrajectory.h>
#include <vector>

#define MAX_JOINT_VEL 0.5

/*! \brief  Simple action server client definition for JointTrajectoryAction */
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

/*! \brief Class to move both hands in sync for graspped object by both hands. 
 *
 *  This class takes in offset in transforms for both ahnds and pose array as objPoseTraj.
 *  It generates joint trajectory for both hands using GetconstraintAwarePositionIK service 
 *  and executes both trajectory in synchronized fashion.
 */
class dualArms
{
public:
    tf::Transform leftWristOffset; /*!< Offset of left arm wrist_roll link from object. */
    tf::Transform rightWristOffset; /*!< Offset of right arm wrist_roll link from object. */
    geometry_msgs::PoseArray objPoseTraj; /*!< Pose trajectory of an object. */
    dualArms(ros::NodeHandle& rh);
    bool genTrajectory();
    bool executeJointTrajectory();
private:
    TrajClient* traj_client_r_; /*!< Right arm trajectory action client. */
    TrajClient* traj_client_l_; /*!< Left arm trajectory action client. */
    ros::ServiceClient ik_client_r_; /*!< Right arm IK client. */
    ros::ServiceClient ik_client_l_; /*!< Left arm IK client. */
    ros::ServiceClient query_client_r_; /*!< Right arm kinematic solver info query client. */
    ros::ServiceClient query_client_l_; /*!< Left arm kinematic solver info query client. */
    ros::ServiceClient filter_trajectory_client_; /*!< Joint trajectory unnormalizer filter client. */
    pr2_controllers_msgs::JointTrajectoryGoal right_goal_, left_goal_; /*!< Joint trajectory goal to execute joint trajectory. */
    std::vector<double> rightJointTrajectory,leftJointTrajectory; /*!< Double linear array to store joint trajectory. */

    void tf2pose(tf::Transform &tf, geometry_msgs::Pose &pose);
    void pose2tf(geometry_msgs::Pose &pose, tf::Transform &tf);
    void get_right_goal_();
    void get_left_goal_();
    void sync_start_times_(void);
    bool call_right_arm_gpik_(void);
    bool call_left_arm_gpik_(void);
    void call_right_joints_unnormalizer_(void);
    void call_left_joints_unnormalizer_(void);
};

#endif // DUALARMS_H
