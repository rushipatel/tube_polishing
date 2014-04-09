#ifndef BASE_CPP
#define BASE_CPP

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#define MB_LGRNM "Base"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Base
{
public:
    Base();
    ~Base();
    bool moveBaseTo(double x, double y);

private:
    MoveBaseClient* _mb_client;

};


Base::Base(){
    _mb_client = new MoveBaseClient("move_base", true);
    while(!_mb_client->waitForServer(ros::Duration(5)) && ros::ok()){
        ROS_WARN_NAMED(MB_LGRNM,"Waiting for move_base action server");
    }
}

Base::~Base(){
    delete _mb_client;
}

bool Base::moveBaseTo(double x, double y){
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.header.frame_id = "/base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    _mb_client->sendGoal(goal);
    _mb_client->waitForResult(ros::Duration(10));
    if(_mb_client->getState()==actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO_NAMED(MB_LGRNM,"Moved base to new position");
        return true;
    }
    else{
        ROS_WARN_NAMED(MB_LGRNM,"Failed to move base to new position!");
        return false;
    }
}


#endif
