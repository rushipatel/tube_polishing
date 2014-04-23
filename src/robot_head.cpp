#include "robotHead.h"

#define RBHD_LGRNM "rbHd"

//! Action client initialization
robotHead::robotHead()
{
    //Initialize the client for the Action interface to the head controller
    point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
    //wait for head controller action server to come up
    while(!point_head_client_->waitForServer(ros::Duration(5.0)))
        ROS_INFO_NAMED(RBHD_LGRNM,"Waiting for the point_head_action server to come up");
}

//! Points the high-def camera frame at a point in a given frame
bool robotHead::lookAt(double x, double y, double z)
{
    //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = "base_link";
    point.point.x = x; point.point.y = y; point.point.z = z;
    goal.target = point;

    //we are pointing the high-def camera frame
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = "high_def_frame";

    //take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(0.5);

    //and go no faster than 1 rad/s
    goal.max_velocity = 1.0;

    //send the goal
    point_head_client_->sendGoal(goal);

    //wait for it to get there (abort after 2 secs to prevent getting stuck)
    point_head_client_->waitForResult(ros::Duration(5));
    actionlib::SimpleClientGoalState state = point_head_client_->getState();
    if(!state.isDone())
    {
        ROS_WARN_NAMED(RBHD_LGRNM,"Couldn't set head to given point");
        return false;
    }
    return true;
}
