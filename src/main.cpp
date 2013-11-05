#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryActionGoal.h>
#include <tabletop_object_detector/TabletopSegmentation.h>
#include <tabletop_object_detector/Table.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/pcl_base.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree_search.h>

#include <pcl/sample_consensus/sac_model_circle.h>

#include <gazebo_msgs/SpawnModel.h>
#include <stdio.h>
#include <Eigen/Eigen>

#include "dualArms.h"
#include "robotHead.h"
#include "tubePerception.h"

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
void generate_normal_marker(pcl::PointCloud<pcl::PointXYZ >::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals, visualization_msgs::Marker &markers)
{
    if(points->points.size()!=normals->points.size())
    {
        ROS_ERROR("Size of Points and Normals is different. May be both are not from same dataset");
        return;
    }

    visualization_msgs::Marker marker;
    geometry_msgs::Point p;

    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 1;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;

    marker.color.r = ((double)rand())/RAND_MAX;
    marker.color.g = ((double)rand())/RAND_MAX;
    marker.color.b = ((double)rand())/RAND_MAX;
    marker.color.a = 1.0;

    marker.header = points->header;

    unsigned int marker_id=1;
    marker.ns = "tube_polishing_node";
    marker.id = marker_id;
    //marker.points.resize(2);

    for(size_t i=0; i<points->points.size(); i++)
    {
        p.x = points->points[i].x;
        p.y = points->points[i].y;
        p.z = points->points[i].z;

        marker.points.push_back(p);
        /*marker.points[1]= p;

        p.x += normals->points[i].normal_x/100;
        p.y += normals->points[i].normal_y/100;
        p.z += normals->points[i].normal_z/100;

        marker.points[0] = p;*/

        //marker.ns = "tube_polishing_node";

        //marker_id++;
        //marker.id = marker_id;
        //markers->markers.push_back(marker);
    }
    markers = marker;
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
    ros::Publisher cloud_pub = rh.advertise<sensor_msgs::PointCloud2>("tube_cloud",2);
    ros::Publisher marker_pub = rh.advertise<visualization_msgs::Marker>("tube_cylinder_markers", 10);
    //ros::ServiceClient spawn_model_client = rh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
    if(!set_planning_scene_diff_client.call(planning_scene_req, planning_scene_res))
    {
        ROS_WARN("Can't get planning scene");
        return -1;
    }
    //ros::service::waitForService("/gazebo/spawn_urdf_model");

    //rotateAroundCenter(rh);

    dualArms dual_arms(rh);
    geometry_msgs::Pose pose;
    pose.position.x = 0.1;
    pose.position.y = -0.6;
    pose.position.z = 0.8;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    dual_arms.moveRightArm(pose);
    pose.position.x = 0.1;
    pose.position.y = 0.6;
    pose.position.z = 0.8;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    dual_arms.moveLeftArm(pose);

    robotHead pr2_head;
    pr2_head.lookAt(0.75,0.0,0.5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr tube_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    //write_kinect_output(rh);
    tabletop_object_detector::TabletopSegmentation seg_srv;
    while(getchar()!='q')
    {
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
                    ROS_INFO("Original cloud size: %d",pc2.height*pc2.width);
                    TubePerception::CloudProcessing cp(pc2);
                    cp.displayCloud(5);
                    //cp.displayAxisPoints(5);
                    //cp.displayLines(60);
                    cp.displayCylinders(0);
                    cp.writeAxisPointsOnFile("/home/wpi_robotics/fuerte_workspace/sandbox/tube_polishing/data/pcd_files/axis_points.pcd");
                }
            }
            else
                ROS_ERROR("Segmentation service returned error %d", seg_srv.response.result);
        }
        else
            ROS_ERROR("Call to segmentation service failed");
    }
  //ros::spin();
  ros::shutdown();
}
