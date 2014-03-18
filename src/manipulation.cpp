#include "manipulation.h"


manipulation::manipulation(ros::NodeHandlePtr nh)
{

    _nh = nh;
    _right_group.reset(new moveit::planning_interface::MoveGroup("right_arm"));
    _left_group.reset(new moveit::planning_interface::MoveGroup("left_arm"));
    _display_publisher = _nh->advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

}

bool manipulation::moveRightArm(geometry_msgs::Pose pose)
{
    _right_group->setPoseTarget(pose);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = _right_group->plan(my_plan);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    if(success)
        _visialize_plan(my_plan);
    _right_group->move();
}

void manipulation::_visialize_plan(moveit::planning_interface::MoveGroup::Plan plan)
{
    //ROS_INFO("Visualizing plan 1 (again)");
    _display_traj.trajectory_start = plan.start_state_;
    _display_traj.trajectory.push_back(plan.trajectory_);
    _display_publisher.publish(_display_traj);
    sleep(5.0);
}

void manipulation::experimental()
{
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.7;
    target_pose1.position.z = 1.0;
    moveRightArm(target_pose1);
}

void manipulation::printState()
{
    ROS_INFO("Right Arm Refererance Frame: %s", _right_group->getPlanningFrame().c_str());
    ROS_INFO("Left Arm Refererance Frame: %s", _right_group->getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", _right_group->getEndEffectorLink().c_str());
}
