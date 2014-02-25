#ifndef DUALARMS_H
#define DUALARMS_H
#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <arm_navigation_msgs/FilterJointTrajectory.h>
#include <vector>

#include <utility.cpp>

#define MAX_JOINT_VEL 0.5

/*! \brief  Simple action server client definition for JointTrajectoryAction */
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

/*! \brief Class to move both hands in sync for graspped object by both hands. 
 *
 *  This class takes in offset in transforms for both hands and objPoseTraj as pose array.
 *  It generates joint trajectory for both hands using GetconstraintAwarePositionIK service 
 *  and executes both trajectory in synchronized fashion.
 */
class dualArms
{
public:
    tf::Transform leftWristOffset; /*!< Offset of left arm wrist_roll link from object. */
    tf::Transform rightWristOffset; /*!< Offset of right arm wrist_roll link from object. */
    geometry_msgs::PoseArray objPoseTraj; /*!< Pose trajectory of an object. */
    dualArms(ros::NodeHandlePtr rh);
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
    void get_current_right_joint_angles(double current_angles[7]);
    void get_current_left_joint_angles(double current_angles[7]);
private:
    TrajClient* traj_client_r_; /*!< Right arm trajectory action client. */
    TrajClient* traj_client_l_; /*!< Left arm trajectory action client. */
    ros::ServiceClient ik_client_r_; /*!< Right arm IK client. */
    ros::ServiceClient ik_client_l_; /*!< Left arm IK client. */
    ros::ServiceClient simple_ik_client_r_; /*!< Left arm IK client. */
    ros::ServiceClient simple_ik_client_l_; /*!< Left arm IK client. */
    ros::ServiceClient query_client_r_; /*!< Right arm kinematic solver info query client. */
    ros::ServiceClient query_client_l_; /*!< Left arm kinematic solver info query client. */
    ros::ServiceClient filter_trajectory_client_; /*!< Joint trajectory unnormalizer filter client. */
    pr2_controllers_msgs::JointTrajectoryGoal right_goal_, left_goal_; /*!< Joint trajectory goal to execute joint trajectory. */
    std::vector<double> right_joint_traj_,left_joint_traj_; /*!< Double linear array to store joint trajectory. */

    void get_right_goal_();
    void get_left_goal_();
    void sync_start_times_(void);
    bool call_right_arm_gpik_(std::vector<double> &right_joint_trajectory);
    bool call_left_arm_gpik_(std::vector<double> &left_joint_trajectory);
    void call_right_joints_unnormalizer_(void);
    void call_left_joints_unnormalizer_(void);
};

#endif // DUALARMS_H
