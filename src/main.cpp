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

    for(int i=0; i<points->points.size(); i++)
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
    pcl::io::savePCDFileASCII("~/home/fuerte_workspace/sandbox/tube_polishing/data/pcd_files/kinect_tube_2.pcd",pcl_cloud);
    ROS_INFO("Kinect output has been written");
}



void ransac_cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr tube_cloud, visualization_msgs::Marker &markers)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (tube_cloud);
    ne.setKSearch (50);
    ne.compute (*normals);

    pcl::PointCloud<pcl::PointXYZINormal> pn;


    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.01);
    seg.setRadiusLimits (0.018, 0.022);
    seg.setInputCloud (tube_cloud);
    seg.setInputNormals (normals);
    seg.setDistanceFromOrigin(0.01);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers, *coeff);
    std::cerr << "Cylinder coefficients: " << *coeff << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(tube_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_out);


    //visualization_msgs::Marker cloud_marker;
//    cloud_marker.action = visualization_msgs::Marker::ADD;
//    cloud_marker.lifetime = ros::Duration();

//    cloud_marker.type = visualization_msgs::Marker::ARROW;
//    cloud_marker.scale.x = coeff->values[6]*2;
//    cloud_marker.scale.y = coeff->values[6]*2;
//    cloud_marker.scale.z = 0;

//    cloud_marker.color.r = ((double)rand())/RAND_MAX;
//    cloud_marker.color.g = ((double)rand())/RAND_MAX;
//    cloud_marker.color.b = ((double)rand())/RAND_MAX;
//    cloud_marker.color.a = 1.0;

//    cloud_marker.header = cloud_out->header;
//    geometry_msgs::Point p;
//    p.x = coeff->values[0];
//    p.y = coeff->values[1];
//    p.z = coeff->values[2];
//    cloud_marker.points.push_back(p);
//    p.x = coeff->values[0]+coeff->values[3];
//    p.y = coeff->values[1]+coeff->values[4];
//    p.z = coeff->values[2]+coeff->values[5];
//    cloud_marker.points.push_back(p);
//    cloud_marker.ns = "tube_polishing_node";
//    cloud_marker.id = 1;

    ROS_INFO("Number of Inliers %d: ",cloud_out->points.size());

//    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.01);
//    pcl::PointXYZ search_point;
//    search_point.x = coeff->values[0];
//    search_point.y = coeff->values[1];
//    search_point.z = coeff->values[2];
//    octree.setInputCloud(tube_cloud);
//    octree.addPointsFromInputCloud();

//    std::vector<int> neighbor_idx;
//    std::vector<float> sqrt_dist;
//    octree.nearestKSearch(search_point,50,neighbor_idx,sqrt_dist);

//    ROS_INFO("Size of neighbour is: %d",neighbor_idx.size());

//    pcl::PointCloud<pcl::PointXYZ>::Ptr neighbor_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointXYZ point;

//    for(int i=0; i<neighbor_idx.size(); i++)
//    {
//        point.x = tube_cloud->points[neighbor_idx[i]].x;
//        point.y = tube_cloud->points[neighbor_idx[i]].y;
//        point.z = tube_cloud->points[neighbor_idx[i]].z;
//        neighbor_cloud->points.push_back(point);
//    }



    generate_normal_marker(tube_cloud, normals, markers);

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
                    cp.processCloud();
                    //cp.displayCloud(5);
                    //cp.displayAxisPoints(5);
                    cp.displayLines(60);
                    cp.displayCylinders(0);

                    //write_kinect_output(rh);
                    //pcl::io::savePCDFileASCII("../data/pcd_files/tube_2.pcd",pcl_cloud);
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
