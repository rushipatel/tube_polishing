#ifndef GRIPPER_H
#define GRIPPER_H

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper
{
public:
    Gripper(std::string whichArm="right_arm");
    bool open(void);
    bool close(void);
    bool setPosition(double position, double effort=50);

private:
    GripperClient *grpr_clnt_;
};


Gripper::Gripper(std::string whichArm)
{
    std::string grpr_controller;
    if(whichArm.compare("right_arm"))
        grpr_controller = "r_gripper_controller/gripper_action";
    else if(whichArm.compare("left_arm"))
        grpr_controller = "l_gripper_controller/gripper_action";
    else
        ROS_ERROR("Gripper - Invalid argument! did you mean 'right_arm' or 'left_arm'?");
    grpr_clnt_ = new GripperClient(grpr_controller.c_str(),true);
    while(!grpr_clnt_->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for r_gripper_controller/gripper_action action server");
}

bool Gripper::open()
{
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.max_effort = -1;
    goal.command.position = 0.08;
    grpr_clnt_->sendGoal(goal);
    grpr_clnt_->waitForResult();
    if(!(grpr_clnt_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    {
        ROS_INFO("Gripper failed to open!");
        return false;
    }
    return true;
}

bool Gripper::close()
{
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.max_effort = 50;
    goal.command.position = 0.001;
    grpr_clnt_->sendGoal(goal);
    grpr_clnt_->waitForResult();
    if(!(grpr_clnt_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    {
        ROS_ERROR("Gripper failed to close!");
        return false;
    }
    return true;
}

bool Gripper::setPosition(double position, double effort)
{
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.max_effort = effort;
    goal.command.position = position;
    grpr_clnt_->sendGoal(goal);
    grpr_clnt_->waitForResult();
    if(!(grpr_clnt_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    {
        ROS_ERROR("Gripper failed to set position!");
        return false;
    }
    return true;
}
#endif // GRIPPER_H

