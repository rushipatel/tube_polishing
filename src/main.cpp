#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <tf/tf.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryActionGoal.h>
#include <tabletop_object_detector/TabletopSegmentation.h>
#include <stdio.h>
#include "dualArms.h"
#define NAME_OF_SEGMENTATION_SRV "/tabletop_segmentation"


void tfToPose(tf::Transform &tf, geometry_msgs::Pose &pose )
{
    pose.position.x = tf.getOrigin().getX();
    pose.position.y = tf.getOrigin().getY();
    pose.position.z = tf.getOrigin().getZ();
    pose.orientation.x = tf.getRotation().getX();
    pose.orientation.y = tf.getRotation().getY();
    pose.orientation.z = tf.getRotation().getZ();
    pose.orientation.w = tf.getRotation().getW();
}

void rotateAroundCenter(ros::NodeHandle rh)
{
    dualArms dual_arms(rh);
    tf::Transform tfBaseObj;
    tf::Transform tfObjWrist_r, tfObjWrist_l;
    geometry_msgs::Pose pose;

    tfObjWrist_r.setOrigin(tf::Vector3(-0.15,-0.2,0));
    tfObjWrist_r.setRotation(tf::Quaternion(0,M_PI/2,0));
    tfObjWrist_l.setOrigin(tf::Vector3(-0.15,0.2,0));
    tfObjWrist_l.setRotation(tf::Quaternion(0,M_PI/2,0));
    dual_arms.rightWristOffset = tfObjWrist_r;
    dual_arms.leftWristOffset = tfObjWrist_l;


    tfBaseObj.setOrigin(tf::Vector3(0.7,0,0.7));
    for(double i=-45; i<=45; i+=1)
    {
        tfBaseObj.setRotation(tf::Quaternion(i*M_PI/180,0,0));
        tfToPose(tfBaseObj, pose);
        dual_arms.objPoseTraj.poses.push_back(pose);
    }
    for(double i=45; i>=-45; i-=1)
    {
        tfBaseObj.setRotation(tf::Quaternion(i*M_PI/180,0,0));
        tfToPose(tfBaseObj, pose);
        dual_arms.objPoseTraj.poses.push_back(pose);
    }
    for(double i=-45; i<=45; i+=1)
    {
        tfBaseObj.setRotation(tf::Quaternion(0,i*M_PI/180,0));
        tfToPose(tfBaseObj, pose);
        dual_arms.objPoseTraj.poses.push_back(pose);
    }
    for(double i=45; i>=-45; i-=1)
    {
        tfBaseObj.setRotation(tf::Quaternion(0,i*M_PI/180,0));
        tfToPose(tfBaseObj, pose);
        dual_arms.objPoseTraj.poses.push_back(pose);
    }
    for(double i=-45; i<=45; i+=1)
    {
        tfBaseObj.setRotation(tf::Quaternion(0,0,i*M_PI/180));
        tfToPose(tfBaseObj, pose);
        dual_arms.objPoseTraj.poses.push_back(pose);
    }
    for(double i=45; i>=-45; i-=1)
    {
        tfBaseObj.setRotation(tf::Quaternion(0,0,i*M_PI/180));
        tfToPose(tfBaseObj, pose);
        dual_arms.objPoseTraj.poses.push_back(pose);
    }

    /*for(int i=0; i<dual_arms.objPoseTraj.poses.size(); i++)
    {
        pose = dual_arms.objPoseTraj.poses[i];
        ROS_INFO("Pose No. %d Origin = %f %f %f",i,pose.position.x, pose.position.y, pose.position.z);
    }*/
    if(!dual_arms.genTrajectory())
        ROS_ERROR("get_collosion_free_ik.cpp: IK Failed");
    else
        dual_arms.executeJointTrajectory();


}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "get_ik");
    ros::NodeHandle rh;
    ros::ServiceClient seg_srv_client = rh.serviceClient<tabletop_object_detector::TabletopSegmentation>(NAME_OF_SEGMENTATION_SRV);
    rotateAroundCenter(rh);

    tabletop_object_detector::TabletopSegmentation seg_srv;
    if(seg_srv_client.call(seg_srv))
    {
        if(seg_srv.response.result==seg_srv.response.SUCCESS)
        {
            ROS_INFO("....OK....");
        }
        else
            ROS_ERROR("Segmentation service returned error %d", seg_srv.response.result);

    }
    else
        ROS_ERROR("Call to segmentation service failed");

  ros::shutdown();
}
