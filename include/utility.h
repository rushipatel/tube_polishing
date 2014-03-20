#ifndef UTILITY_H
#define UTILITY_H

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
//#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>

geometry_msgs::Pose tf2pose(tf::Transform &t)
{
    geometry_msgs::Pose p;
    tf::Vector3 v = t.getOrigin();
    tf::Quaternion q = t.getRotation();
    p.position.x = v.x();
    p.position.y = v.y();
    p.position.z = v.z();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();

    return p;
}

tf::Transform pose2tf(geometry_msgs::Pose &p)
{
    tf::Transform t;
    tf::Quaternion q;
    q.setX(p.orientation.x);
    q.setY(p.orientation.y);
    q.setZ(p.orientation.z);
    q.setW(p.orientation.w);
    t.setOrigin(tf::Vector3(p.position.x, p.position.y, p.position.z));
    t.setRotation(q);
    return t;
}

/*std::string armNavigationErrorCodeToString(arm_navigation_msgs::ArmNavigationErrorCodes &error_code)
{
    std::string result;
    if(error_code.val == error_code.PLANNING_FAILED)
      result = "Planning failed";
    else if(error_code.val == error_code.SUCCESS)
      result = "Success";
    else if(error_code.val == error_code.TIMED_OUT)
      result = "Timed out";
    else if (error_code.val == error_code.START_STATE_IN_COLLISION)
      result = "Start state in collision";
    else if (error_code.val == error_code.START_STATE_VIOLATES_PATH_CONSTRAINTS)
      result = "Start state violates path constraints";
    else if (error_code.val == error_code.GOAL_IN_COLLISION)
      result = "Goal in collision";
    else if (error_code.val == error_code.GOAL_VIOLATES_PATH_CONSTRAINTS)
      result = "Goal violates path constraints";
    else if (error_code.val == error_code.INVALID_ROBOT_STATE)
      result = "Initial robot state invalid";
    else if (error_code.val == error_code.INCOMPLETE_ROBOT_STATE)
      result = "Initial robot state incomplete";
    else if (error_code.val == error_code.INVALID_PLANNER_ID)
      result = "Invalid planner id";
    else if (error_code.val == error_code.INVALID_NUM_PLANNING_ATTEMPTS)
      result = "Invalid num planning attempts (must be > 0)";
    else if (error_code.val == error_code.INVALID_ALLOWED_PLANNING_TIME)
      result = "Invalid allowed planning time (must be > 0)";
    else if (error_code.val == error_code.INVALID_GROUP_NAME)
      result = "Invalid group name for planning";
    else if (error_code.val == error_code.INVALID_GOAL_JOINT_CONSTRAINTS)
      result = "Invalid goal joint constraints";
    else if (error_code.val == error_code.INVALID_GOAL_POSITION_CONSTRAINTS)
      result = "Invalid goal position constraints";
    else if (error_code.val == error_code.INVALID_GOAL_ORIENTATION_CONSTRAINTS)
      result = "Invalid goal orientation constraints";
    else if (error_code.val == error_code.INVALID_PATH_JOINT_CONSTRAINTS)
      result = "Invalid path joint constraints";
    else if (error_code.val == error_code.INVALID_PATH_POSITION_CONSTRAINTS)
      result = "Invalid path position constraints";
    else if (error_code.val == error_code.INVALID_PATH_ORIENTATION_CONSTRAINTS)
      result = "Invalid path orientation constraints";
    else if (error_code.val == error_code.INVALID_TRAJECTORY)
      result = "Invalid trajectory";
    else if (error_code.val == error_code.INVALID_INDEX)
      result = "Invalid index for trajectory check";
    else if (error_code.val == error_code.JOINT_LIMITS_VIOLATED)
      result = "Joint limits violated";
    else if (error_code.val == error_code.PATH_CONSTRAINTS_VIOLATED)
      result = "Path constraints violated";
    else if (error_code.val == error_code.COLLISION_CONSTRAINTS_VIOLATED)
      result = "Collision constraints violated";
    else if (error_code.val == error_code.GOAL_CONSTRAINTS_VIOLATED)
      result = "Goal constraints violated";
    else if (error_code.val == error_code.JOINTS_NOT_MOVING)
      result = "Joints not moving - robot may be stuck";
    else if (error_code.val == error_code.TRAJECTORY_CONTROLLER_FAILED)
      result = "Trajectory controller failed";
    else if (error_code.val == error_code.FRAME_TRANSFORM_FAILURE)
      result = "Frame transform failed";
    else if (error_code.val == error_code.COLLISION_CHECKING_UNAVAILABLE)
      result = "Collision checking unavailable";
    else if (error_code.val == error_code.ROBOT_STATE_STALE)
      result = "Robot state is not being updated";
    else if (error_code.val == error_code.SENSOR_INFO_STALE)
      result = "Sensor information is not being updated";
    else if (error_code.val == error_code.NO_IK_SOLUTION)
      result = "Inverse kinematics solution was not found";
    else if (error_code.val == error_code.IK_LINK_IN_COLLISION)
      result = "Inverse kinematics link was in collision";
    else if (error_code.val == error_code.INVALID_LINK_NAME)
      result = "Invalid link name";
    else if (error_code.val == error_code.NO_FK_SOLUTION)
      result = "No forward kinematics solution";
    else if (error_code.val == error_code.KINEMATICS_STATE_IN_COLLISION)
      result = "Current robot state is in collision";
    else if (error_code.val == error_code.INVALID_TIMEOUT)
      result = "Time given for planning invalid (must be > 0)";
    else
      result = "Unknown error code";
    return result;
}*/

#endif
