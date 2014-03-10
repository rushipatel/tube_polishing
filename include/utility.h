#ifndef UTILITY_H
#define UTILITY_H

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

geometry_msgs::Pose tfToPose(tf::Transform t)
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

tf::Transform poseToTf(geometry_msgs::Pose p)
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

#endif
