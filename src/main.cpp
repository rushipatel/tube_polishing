#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <tf/tf.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryActionGoal.h>
#include <tabletop_object_detector/TabletopSegmentation.h>
#include <tabletop_object_detector/Table.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/io/pcd_io.h>
#include <stdio.h>
#include <Eigen/Eigen>
#include "dualArms.h"
#include "robotHead.h"
#define SEGMENTATION_SRV "/tabletop_segmentation"
#define SET_PLANNING_SCENE_DIFF_NAME "/environment_server/set_planning_scene_diff"


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
    /*for(double i=45; i>=-45; i-=1)
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
    }*/

    /*for(int i=0; i<dual_arms.objPoseTraj.poses.size(); i++)
    {
        pose = dual_arms.objPoseTraj.poses[i];
        ROS_INFO("Pose No. %d Origin = %f %f %f",i,pose.position.x, pose.position.y, pose.position.z);
    }*/
    if(!dual_arms.genTrajectory())
        ROS_ERROR("IK Failed");
    else
        dual_arms.executeJointTrajectory();

}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "tube_polishing");
    ros::NodeHandle rh;
    ros::ServiceClient seg_srv_client = rh.serviceClient<tabletop_object_detector::TabletopSegmentation>(SEGMENTATION_SRV);
    ros::ServiceClient set_planning_scene_diff_client = rh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
    arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
    if(!set_planning_scene_diff_client.call(planning_scene_req, planning_scene_res))
    {
        ROS_WARN("Can't get planning scene");
        return -1;
    }
    //rotateAroundCenter(rh);
    geometry_msgs::Pose pose;
    pose.position.x = 0.1;
    pose.position.y = -0.6;
    pose.position.z = 0.8;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    dualArms dual_arms(rh);
    dual_arms.moveRightArm(pose);

    pose.position.x = 0.1;
    pose.position.y = 0.6;
    pose.position.z = 0.8;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    dual_arms.moveLeftArm(pose);
    double j_val[7];
    dual_arms.get_current_right_joint_angles(j_val);
    ROS_INFO("Right: %f %f %f %f %f %f %f",j_val[0],j_val[1],j_val[2],j_val[3],j_val[4],j_val[5],j_val[6]);
    dual_arms.get_current_left_joint_angles(j_val);
    ROS_INFO("Left: %f %f %f %f %f %f %f",j_val[0],j_val[1],j_val[2],j_val[3],j_val[4],j_val[5],j_val[6]);
    robotHead pr2_head;
    pr2_head.lookAt(1.0,0.0,0.5);

    tabletop_object_detector::TabletopSegmentation seg_srv;
    if(seg_srv_client.call(seg_srv))
    {
        if(seg_srv.response.result==seg_srv.response.SUCCESS)
        {
            ROS_INFO("....OK....");
            for(unsigned int i=0; i<seg_srv.response.clusters.size(); i++)
            {
                sensor_msgs::PointCloud pc;
                pc = seg_srv.response.clusters[i];
                sensor_msgs::PointCloud2 pc2;
                sensor_msgs::convertPointCloudToPointCloud2(pc, pc2);
                ROS_INFO("Hight: %d     Width: %d",pc2.height, pc2.width);
                pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
                pcl::fromROSMsg(pc2,pcl_cloud);
                pcl::io::savePCDFileASCII("/home/wpi_robotics/fuerte_workspace/sandbox/tube_polishing/data/pcd_files/tube_3.pcd",pcl_cloud);
            }

        }
        else
            ROS_ERROR("Segmentation service returned error %d", seg_srv.response.result);
    }
    else
        ROS_ERROR("Call to segmentation service failed");

  ros::shutdown();
}
