#ifndef DUALARMS2_H
#define DUALARMS2_H
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

#include <moveit_msgs/GetCartesianPath.h>
#include <moveit_msgs/GetCartesianPathRequest.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/move_group_interface/move_group.h>

class dualArms2
{
public:
    dualArms2();
};

#endif // DUALARMS2_H
