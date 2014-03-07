#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryActionGoal.h>
#include <tabletop_object_detector/TabletopSegmentation.h>
#include <tabletop_object_detector/Table.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometric_shapes/shapes.h>

#include <gazebo_msgs/SpawnModel.h>
#include <stdio.h>
#include <Eigen/Eigen>

#include "utility.h"
#include "tubeManipulation.h"
//#include "robotHead.h"
//#include "tubePerception.h"
//#include "tubeGrasp.h"
//#include "manipAnalysis.h"
//#include "gripper.h"

#define SEGMENTATION_SRV "/tabletop_segmentation"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "tube_polishing");
    ros::NodeHandlePtr rh(new ros::NodeHandle);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient seg_srv_client = rh->serviceClient<tabletop_object_detector::TabletopSegmentation>(SEGMENTATION_SRV);
    //ros::Publisher cloud_pub = rh->advertise<sensor_msgs::PointCloud2>("tube_cloud",2);
    //ros::Publisher marker_pub = rh->advertise<visualization_msgs::Marker>("tube_cylinder_markers", 10);
    ros::Publisher pose_pub = rh->advertise<geometry_msgs::PoseStamped>("/tube_polishing/work_traj_pose",10);
    ros::Publisher marker_pub = rh->advertise<visualization_msgs::Marker>("/tube_polishing/marker", 2);
    ros::Publisher tube_marker_pub = rh->advertise<visualization_msgs::MarkerArray>("/tube_polishing/tube_marker", 2);
    ros::Publisher grasp_marker_pub = rh->advertise<visualization_msgs::MarkerArray>("/tube_polishing/grasp_marker", 2);
    //ros::ServiceClient spawn_model_client = rh->serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_model");

    TubeManipulation::CollisionCheck collision_check(rh);
    collision_check.printState();

    /*TubeManipulation manip(rh);
    geometry_msgs::Pose pose;
    pose.position.x = 0.1;
    pose.position.y = -0.6;
    pose.position.z = 0.8;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    manip.moveRightArm(pose);
    pose.position.x = 0.1;
    pose.position.y = 0.6;
    pose.position.z = 0.8;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    manip.moveLeftArm(pose);
    std::vector<double> right_joints, left_joints;
    manip.isStateValid(right_joints,left_joints);

    robotHead pr2_head;
    pr2_head.lookAt(0.75,0.0,0.5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr tube_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    //write_kinect_output(rh);
    tabletop_object_detector::TabletopSegmentation seg_srv;
    geometry_msgs::PoseArray posearray;
    TubePerception::Tube::Ptr tube;
    visualization_msgs::MarkerArray marker_array, grasp_marker;
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

                    tube.reset(new TubePerception::Tube(pc2));
                    TubePerception::CloudProcessing cp(tube);
                    tube->getCylinderMarker(marker_array);
                    tube_marker_pub.publish(marker_array);
                    tube->getCylinderPoses(posearray);

                    //cp.displayCloud();
                    //cp.displayAxisPoints();
                    //cp.displayLines();
                    //cp.displayCylinders();
                    //cp.displayCylindersInLocalFrame();
                    //cp.writeAxisPointsOnFile("/home/wpi_robotics/fuerte_workspace/sandbox/tube_polishing/data/pcd_files/axis_points.pcd");

                    //TubeGrasp::GraspArray::Ptr grasp_array(new TubeGrasp::GraspArray);
                    //TubeGrasp::GraspAnalysis grasp_analysis(grasp_array);
                    //grasp_analysis.generateGrasps(tube);
                    //cp.displayCylinders(TubeGrasp::displayGrasps(grasp_array));
                    //cp.dispalyWorkTraj();
                    TubeGrasp::GraspAnalysis ga(tube, rh);
                    geometry_msgs::Pose work_pose;
                    work_pose.position.x = 0.7;
                    work_pose.position.y = 0.0;
                    work_pose.position.z = 0.8;
                    work_pose.orientation.x = 0.0;
                    work_pose.orientation.y = 0.0;
                    work_pose.orientation.z = 0.0;
                    work_pose.orientation.w = 1.0;
                    ga.setWorkPose(work_pose);
                    ga.analyze();
                    ga.getGraspMarker(grasp_marker);
                    grasp_marker_pub.publish(grasp_marker);
                    geometry_msgs::Pose pick_pose;
                    ga.pickUpTube(pick_pose);
                    geometry_msgs::PoseStamped ps;
                    ps.header.frame_id = "base_link";
                    ps.header.stamp = ros::Time::now();
                    ps.pose = pick_pose;
                    pose_pub.publish(ps);
                    tube->setPose(pick_pose);
                    tube->getCylinderMarker(marker_array);
                    tube_marker_pub.publish(marker_array);
                    //posearray = ga._tube_traj;
                    posearray = ga.grasp_pose_array;
                    //posearray.poses.push_back(tube->getPose());
                    //posearray.poses.push_back(tube->getPose());
                    //posearray.poses.push_back(tube->getPose());
                    //ROS_INFO_STREAM("Size of trajectory = "<<posearray.poses.size());
                }
            }
            else
                ROS_ERROR("Segmentation service returned error %d", seg_srv.response.result);
        }
        else
            ROS_ERROR("Call to segmentation service failed");
    }

    geometry_msgs::Pose tube_pose;
    tube_pose.orientation.x = 0;
    tube_pose.orientation.y = 0;
    tube_pose.orientation.z = 0;
    tube_pose.orientation.w = 1;
    tube_pose.position.x = 0;
    tube_pose.position.y = 0;
    tube_pose.position.z = 0;
    //tube->setPose(tube_pose);
    tube->getCylinderMarker(marker_array);
    tube_marker_pub.publish(marker_array);

    geometry_msgs::PoseStamped posestamped;
    posestamped.header.frame_id = "/base_link";
    int cnt = -1;
    while (getchar()!='q')
    {
        cnt++;
        if(cnt>=posearray.poses.size())
            cnt=0;
        posestamped.pose = posearray.poses[cnt];
        posestamped.header.stamp = ros::Time::now();
        pose_pub.publish(posestamped);
        ROS_INFO_STREAM(" "<<cnt);
        //tube->setPose(posearray.poses[cnt]);
        tube->getCylinderMarker(marker_array);
        tube_marker_pub.publish(marker_array);
    }
    //while (getchar()!='q');
    //ros::spin();


  ros::shutdown();*/
}
