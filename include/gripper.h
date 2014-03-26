#ifndef GRIPPER_H
#define GRIPPER_H

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperEventDetectorAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperFindContactAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper
{
public:
    Gripper();
    bool openRightGripper(void);
    bool openLeftGripper(void);
    bool closeRightGripper(void);
    bool closeLeftGripper(void);
    bool setRightGripperPosition(double position, double effort=50);
    bool setLeftGripperPosition(double position, double effort=50);

private:
    GripperClient* _grpr_clnt_r;
    GripperClient* _grpr_clnt_l;
};


Gripper::Gripper()
{
    _grpr_clnt_r = new GripperClient("r_gripper_controller/gripper_action",true);
    while(!_grpr_clnt_r->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for r_gripper_controller/gripper_action action server");

    _grpr_clnt_l = new GripperClient("l_gripper_controller/gripper_action",true);
    while(!_grpr_clnt_l->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for l_gripper_controller/gripper_action action server");
}

bool Gripper::openRightGripper()
{
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.max_effort = -1;
    goal.command.position = 0.08;
    _grpr_clnt_r->sendGoal(goal);
    _grpr_clnt_r->waitForResult(ros::Duration(30));
    if(!(_grpr_clnt_r->getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    {
        ROS_INFO("Gripper - Right gripper failed to open!");
        return false;
    }
    return true;
}

bool Gripper::openLeftGripper()
{
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.max_effort = -1;
    goal.command.position = 0.08;
    _grpr_clnt_l->sendGoal(goal);
    _grpr_clnt_l->waitForResult(ros::Duration(30));
    if(!(_grpr_clnt_l->getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    {
        ROS_INFO("Gripper - Left gripper failed to open!");
        return false;
    }
    return true;
}

bool Gripper::closeRightGripper()
{
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.max_effort = 50;
    goal.command.position = 0.001;
    _grpr_clnt_r->sendGoal(goal);
    _grpr_clnt_r->waitForResult();
    if(!(_grpr_clnt_r->getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    {
        ROS_ERROR("Gripper - Right gripper failed to close!");
        return false;
    }
    return true;
}

bool Gripper::closeLeftGripper()
{
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.max_effort = 50;
    goal.command.position = 0.001;
    _grpr_clnt_l->sendGoal(goal);
    _grpr_clnt_l->waitForResult();
    if(!(_grpr_clnt_l->getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    {
        ROS_ERROR("Gripper - Right gripper failed to close!");
        return false;
    }
    return true;
}

bool Gripper::setRightGripperPosition(double position, double effort)
{
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.max_effort = effort;
    goal.command.position = position;
    _grpr_clnt_r->sendGoal(goal);
    _grpr_clnt_r->waitForResult();
    if(!(_grpr_clnt_r->getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    {
        ROS_ERROR("Gripper - Right gripper failed to set position!");
        return false;
    }
    return true;
}

bool Gripper::setLeftGripperPosition(double position, double effort)
{
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.max_effort = effort;
    goal.command.position = position;
    _grpr_clnt_l->sendGoal(goal);
    _grpr_clnt_l->waitForResult();
    if(!(_grpr_clnt_l->getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    {
        ROS_ERROR("Gripper - Left gripper failed to set position!");
        return false;
    }
    return true;
}
#endif // GRIPPER_H

