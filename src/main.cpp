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

#include <gazebo_msgs/SpawnModel.h>
#include <stdio.h>
#include <Eigen/Eigen>

#include "dualArms.h"
#include "robotHead.h"
#include "tubePerception.h"
#include "tubeGrasp.h"
#include "manip_analysis.cpp"
#include "utility.cpp"

#define SEGMENTATION_SRV "/tabletop_segmentation"
#define SET_PLANNING_SCENE_DIFF_NAME "/environment_server/set_planning_scene_diff"

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
        pose = tf2pose(tfBaseObj);
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

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient seg_srv_client = rh.serviceClient<tabletop_object_detector::TabletopSegmentation>(SEGMENTATION_SRV);
    ros::ServiceClient set_planning_scene_diff_client = rh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
    //ros::Publisher cloud_pub = rh.advertise<sensor_msgs::PointCloud2>("tube_cloud",2);
    //ros::Publisher marker_pub = rh.advertise<visualization_msgs::Marker>("tube_cylinder_markers", 10);
    ros::Publisher pose_pub = rh.advertise<geometry_msgs::PoseStamped>("/tube_polishing/work_traj_pose",10);
    ros::Publisher marker_pub = rh.advertise<visualization_msgs::Marker>("/tube_polishing/marker", 2);
    ros::Publisher marker_array_pub = rh.advertise<visualization_msgs::MarkerArray>("/tube_polishing/marker_array", 2);
    //ros::ServiceClient spawn_model_client = rh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_model");
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
    geometry_msgs::PoseArray posearray;
    TubePerception::Tube::Ptr tube;
    visualization_msgs::MarkerArray marker_array;
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
                    marker_array_pub.publish(marker_array);
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
                    TubeGrasp::GraspAnalysis ga(tube);
                    geometry_msgs::Pose work_pose;
                    work_pose.position.x = 1.1;
                    work_pose.position.y = 0.0;
                    work_pose.position.z = 0.8;
                    work_pose.orientation.x = 0.0;
                    work_pose.orientation.y = 0.0;
                    work_pose.orientation.z = 0.0;
                    work_pose.orientation.w = 1.0;
                    ga.setWorkPose(work_pose);
                    ga.generateWorkTrajectory();
                    marker_pub.publish(ga.vismsg_workNormalsX);
                    marker_pub.publish(ga.vismsg_workNormalsY);
                    marker_pub.publish(ga.vismsg_workNormalsZ);
                    posearray = ga.tube_traj_;
                    posearray.poses.push_back(tube->getPose());
                    posearray.poses.push_back(tube->getPose());
                    posearray.poses.push_back(tube->getPose());
                    posearray.poses.push_back(work_pose);
                    posearray.poses.push_back(work_pose);
                    posearray.poses.push_back(work_pose);
                    posearray.poses.push_back(work_pose);
                    posearray.poses.push_back(work_pose);
                    //ROS_INFO_STREAM("Size of trajectory = "<<posearray.poses.size());

                }
            }
            else
                ROS_ERROR("Segmentation service returned error %d", seg_srv.response.result);
        }
        else
            ROS_ERROR("Call to segmentation service failed");
    }


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
        tube->setPose(posearray.poses[cnt]);
        tube->getCylinderMarker(marker_array);
        marker_array_pub.publish(marker_array);
    }
    //while (getchar()!='q');
    //ros::spin();
    ManipAnalysis ma("right_arm", rh);
    ma.setRotationAxis(tf::Vector3(1,0,0));
    ma.setForceVec(tf::Vector3(1,0,0));
    std::vector<double> q;
    q.resize(7);
    q[0] = -1.5;
    q[1] =  0.0;
    q[2] =  0.0;
    q[3] =  0.0;
    q[4] = -0.15;
    q[5] = -0.1;
    q[6] =  0.0;
    ROS_INFO_STREAM("FORCE MATRIC   : "<<ma.getForceMatric(q));
    ROS_INFO_STREAM("ROTATION MATRIC: "<<ma.getRotationMatric(q));
    ROS_INFO_STREAM("K              : "<<ma.getManipIndex(q));


  ros::shutdown();
}
