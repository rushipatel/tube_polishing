#ifndef GRIPPER_H
#define GRIPPER_H

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandActionResult.h>
//#include <pr2_gripper_sensor_msgs/PR2GripperGrabAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/JointControllerState.h>

#define GRPR_LGRNM "grpr"

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper
{
public:
    Gripper(ros::NodeHandlePtr nh);
    ~Gripper();
    bool openRightGripper(void);
    bool openLeftGripper(void);
    bool closeRightGripper(void);
    bool closeLeftGripper(void);
    bool setRightGripperPosition(double position, double effort);
    bool setLeftGripperPosition(double position, double effort);
    void setTimeoutValue(int sec);
    void setMinimumError(double err);
    typedef boost::shared_ptr<Gripper> Ptr;
private:
    ros::NodeHandlePtr _nh;
    GripperClient* _grpr_clnt_r;
    GripperClient* _grpr_clnt_l;
    int _timeout;
    double _joint_state_err;
};


Gripper::Gripper(ros::NodeHandlePtr nh)
{
    _nh = nh;
    _grpr_clnt_r = new GripperClient("r_gripper_controller/gripper_action",true);
    while(!_grpr_clnt_r->waitForServer(ros::Duration(5.0)))
        ROS_INFO_NAMED(GRPR_LGRNM,"Waiting for r_gripper_controller/gripper_action action server");

    _grpr_clnt_l = new GripperClient("l_gripper_controller/gripper_action",true);
    while(!_grpr_clnt_l->waitForServer(ros::Duration(5.0)))
        ROS_INFO_NAMED(GRPR_LGRNM,"Waiting for l_gripper_controller/gripper_action action server");
    _timeout = 15;
    _joint_state_err = 0.003;
}

Gripper::~Gripper(){
    delete _grpr_clnt_r;
    delete _grpr_clnt_l;
}

void Gripper::setTimeoutValue(int sec){
    _timeout = sec;
}

void Gripper::setMinimumError(double err){
    _joint_state_err = err;
}

bool Gripper::openRightGripper()
{
    double effort = -1;
    double position = 0.8;
    return setRightGripperPosition(position, effort);
}

bool Gripper::openLeftGripper()
{
    double effort = -1;
    double position = 0.8;
    return setLeftGripperPosition(position, effort);
}

bool Gripper::closeRightGripper()
{
    double effort = 500;
    double position = 0.001;
    return setRightGripperPosition(position, effort);
}

bool Gripper::closeLeftGripper()
{
    double effort = 500;
    double position = 0.001;
    return setLeftGripperPosition(position, effort);
}

bool Gripper::setRightGripperPosition(double position, double effort)
{
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.max_effort = effort;
    goal.command.position = position;

    _grpr_clnt_r->sendGoal(goal);

    _grpr_clnt_r->waitForResult(ros::Duration(_timeout));

    ros::Time stop_time = ros::Time::now() + ros::Duration(_timeout);
    std::vector<double> err_dot(10);
    for(unsigned int i=0; i<err_dot.size(); i++){
        err_dot[i] = 0;
    }
    double prev_err = 0;
    while(ros::ok() && ros::Time::now()<stop_time){
        pr2_controllers_msgs::JointControllerStateConstPtr state =
                ros::topic::waitForMessage<pr2_controllers_msgs::JointControllerState>
                ("r_gripper_controller/state", *_nh);
        if(state->error<_joint_state_err){
            ROS_DEBUG_NAMED(GRPR_LGRNM,"Gripper set before timeout");
            return true;
        }
        err_dot[err_dot.size()-1] = state->error - prev_err;
        prev_err = state->error;
        std::rotate(err_dot.rbegin(), err_dot.rbegin()+1, err_dot.rend());
        double sum=0;
        for(unsigned int i=0; i<err_dot.size(); i++){
            sum += err_dot[i];
        }

        //ROS_INFO("right SUM : %f",sum);

        //if in stall condition and error is somewhat acceptable
        if(sum<0.01 /*&& (state->error < (_joint_state_err*4))*/){
            ROS_WARN_NAMED(GRPR_LGRNM,"Right gripper stalled!");
            return true;
        }
        ROS_DEBUG_NAMED(GRPR_LGRNM,"Right gripper joint state error: %f",state->error);
        ros::Duration(0.2).sleep();
    }

//    if(!(_grpr_clnt_r->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)){
//        ROS_ERROR("Gripper - Right gripper failed to set position!");
//        return false;
//    }
    ROS_WARN_NAMED(GRPR_LGRNM,"Right gripper action timeout (%d Seconds)",_timeout);
//    return false;
    return true;
}

bool Gripper::setLeftGripperPosition(double position, double effort)
{
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.max_effort = effort;
    goal.command.position = position;
    _grpr_clnt_l->sendGoal(goal);

    _grpr_clnt_l->waitForResult(ros::Duration(_timeout));

    ros::Time stop_time = ros::Time::now() + ros::Duration(_timeout);
    std::vector<double> err_dot(10);
    for(unsigned int i=0; i<err_dot.size(); i++){
        err_dot[i] = 0;
    }
    while(ros::ok() && ros::Time::now()<stop_time){
        pr2_controllers_msgs::JointControllerStateConstPtr state =
                ros::topic::waitForMessage<pr2_controllers_msgs::JointControllerState>
                ("l_gripper_controller/state", *_nh);
        if(state->error<_joint_state_err){
            ROS_DEBUG_NAMED(GRPR_LGRNM,"Gripper set before timeout");
            return true;
        }
        err_dot[err_dot.size()-1] = state->error - err_dot[0];
        std::rotate(err_dot.rbegin(), err_dot.rbegin()+1, err_dot.rend());
        double sum=0;
        for(unsigned int i=0; i<err_dot.size(); i++){
            sum += err_dot[i];
        }
        //ROS_INFO("SUM : %f",sum);
        //if in stall condition and error is somewhat acceptable
        if(sum<0.01 /*&& (state->error < (_joint_state_err*4))*/){
            ROS_WARN_NAMED(GRPR_LGRNM,"Left gripper stalled!");
            return true;
        }
        ROS_DEBUG_NAMED(GRPR_LGRNM,"Left gripper joint state error: %f",state->error);
        ros::Duration(0.2).sleep();
    }

//    if(!(_grpr_clnt_l->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)){
//        ROS_ERROR("Gripper - Left gripper failed to set position!");
//        return false;
//    }

    ROS_WARN_NAMED(GRPR_LGRNM,"Left gripper action timeout (%d Seconds)",_timeout);
    //return false;
    return true;
}
#endif // GRIPPER_H

