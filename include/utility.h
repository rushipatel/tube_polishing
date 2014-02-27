#ifndef UTILITY_CPP
#define UTILITY_CPP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

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
    tf::Vector3 v;
    tf::Quaternion q;
    v.setX(p.position.x);
    v.setY(p.position.y);
    v.setZ(p.position.z);
    q.setX(p.orientation.x);
    q.setY(p.orientation.y);
    q.setZ(p.orientation.z);
    q.setW(p.orientation.w);
    t.setOrigin(v);
    t.setRotation(q);
    return t;
}

#endif
