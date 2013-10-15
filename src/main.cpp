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

#include <gazebo_msgs/SpawnModel.h>
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

void write_kinect_output(ros::NodeHandle &nh)
{
    sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("head_mount_kinect/depth/points", nh, ros::Duration(3.0));
    tf::TransformListener listener_;
    if (!recent_cloud)
        ROS_ERROR("no point_cloud2 has been received");

    //convert cloud to processing_frame_ (usually base_link)
    sensor_msgs::PointCloud old_cloud;
    sensor_msgs::PointCloud2 converted_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud (*recent_cloud, old_cloud);
    int current_try=0, max_tries = 3;
    while (1)
    {
        bool transform_success = true;
        try
        {
            listener_.transformPointCloud("base_link", old_cloud, old_cloud);
        }
        catch (tf::TransformException ex)
        {
            transform_success = false;
            if (++current_try >= max_tries)
            {
                ROS_ERROR("Failed to transform cloud from frame %s into frame %s in %d attempt(s)", old_cloud.header.frame_id.c_str(),"base_link", current_try);
                break;
            }
            ROS_DEBUG("Failed to transform point cloud, attempt %d out of %d, exception: %s", current_try, max_tries, ex.what());
            //sleep a bit to give the listener a chance to get a new transform
            ros::Duration(0.1).sleep();
        }
        if (transform_success) break;
        sensor_msgs::convertPointCloudToPointCloud2 (old_cloud, converted_cloud);
        ROS_INFO_STREAM("Input cloud converted to " << "/base_link" << " frame after " );
    }

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(converted_cloud,pcl_cloud);
    pcl::io::savePCDFileASCII("/home/wpi_robotics/fuerte_workspace/sandbox/tube_polishing/data/pcd_files/kinect_tube_2.pcd",pcl_cloud);
    ROS_INFO("Kinect output has been written");
}

void runRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr tube_cloud_ptr, visualization_msgs::Marker& cloud_marker)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::Normal>::Ptr tube_normals_ptr (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_cylinder_ptr (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder_ptr (new pcl::PointIndices);

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (tube_cloud_ptr);
    ne.setKSearch (50);
    ne.compute (*tube_normals_ptr);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.01);
    seg.setRadiusLimits (0.018, 0.022);
    seg.setInputCloud (tube_cloud_ptr);
    seg.setInputNormals (tube_normals_ptr);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder_ptr, *coefficients_cylinder_ptr);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder_ptr << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cyl_out_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(tube_cloud_ptr);
    extract.setIndices(inliers_cylinder_ptr);
    extract.setNegative(false);
    extract.filter(*cyl_out_cloud_ptr);

    //visualization_msgs::Marker cloud_marker;
    cloud_marker.action = visualization_msgs::Marker::ADD;
    cloud_marker.lifetime = ros::Duration();

    cloud_marker.type = visualization_msgs::Marker::ARROW;
    cloud_marker.scale.x = coefficients_cylinder_ptr->values[6]*2;
    cloud_marker.scale.y = coefficients_cylinder_ptr->values[6]*2;
    cloud_marker.scale.z = 0;

    cloud_marker.color.r = ((double)rand())/RAND_MAX;
    cloud_marker.color.g = ((double)rand())/RAND_MAX;
    cloud_marker.color.b = ((double)rand())/RAND_MAX;
    cloud_marker.color.a = 1.0;

    cloud_marker.header = cyl_out_cloud_ptr->header;
    geometry_msgs::Point p;
    p.x = coefficients_cylinder_ptr->values[0];
    p.y = coefficients_cylinder_ptr->values[1];
    p.z = coefficients_cylinder_ptr->values[2];
    cloud_marker.points.push_back(p);
    p.x = coefficients_cylinder_ptr->values[0]+coefficients_cylinder_ptr->values[3];
    p.y = coefficients_cylinder_ptr->values[1]+coefficients_cylinder_ptr->values[4];
    p.z = coefficients_cylinder_ptr->values[2]+coefficients_cylinder_ptr->values[5];
    cloud_marker.points.push_back(p);
    cloud_marker.ns = "tube_polishing_node";
    cloud_marker.id = 1;

    ROS_INFO("Number of Inliers %d: ",cyl_out_cloud_ptr->points.size());

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
                    ROS_INFO("Hight: %d     Width: %d",pc2.height, pc2.width);
                    pcl::fromROSMsg(pc2,pcl_cloud);
                    //write_kinect_output(rh);
                    pcl::io::savePCDFileASCII("/home/wpi_robotics/fuerte_workspace/sandbox/tube_polishing/data/pcd_files/tube_2.pcd",pcl_cloud);
                    pcl::visualization::CloudViewer cloud_viewer("simple_cloud_viewer");
                    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr;
                    pcl_cloud_ptr = pcl_cloud.makeShared();
                    tube_cloud_ptr = pcl_cloud.makeShared();
                    cloud_viewer.showCloud(pcl_cloud_ptr);
                    //while(!cloud_viewer.wasStopped()){}
                }
                visualization_msgs::Marker m;
                runRANSAC(pcl_cloud.makeShared(),m);
                marker_pub.publish(m);
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
