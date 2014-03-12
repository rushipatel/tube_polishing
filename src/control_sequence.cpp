#include "controlSequence.h"


ControlSequence::ControlSequence(ros::NodeHandlePtr nh)
{
    _nh = nh;
    _seg_srv_client = _nh->serviceClient<tabletop_object_detector::TabletopSegmentation>("/tabletop_segmentation");
    _arms.reset(new TubeManipulation::Arms(_nh));
    _tube.reset(new TubePerception::Tube);
    _tube_mrkr_pub = _nh->advertise<visualization_msgs::MarkerArray>("/tube_polishing/tube_marker", 2);

    INIT = 1;
    AT_APRCH = 2;
    AT_PICKUP = 3;
    GRASPED = 4;
    STAGING = 5;
    MCNING = 6;
    BAD = -1;
    GOOD = 0;
    state = GOOD;
    //_cloud_process.reset(new TubePerception::CloudProcessing());
    //_grasp_analysis.reset(new TubeGrasp::GraspAnalysis);
}

ControlSequence::~ControlSequence()
{
    ;
}

bool ControlSequence::initialize()
{
    bool good = true;

    if(!_set_cloud_capture_posture())
    {
        ROS_ERROR("ControlSequence - Unable to intitialize capture posture");
        good = false;
        state = BAD;
        return good;
    }

    _clusters.clear();
    if(!_get_segmented_cloud()) //fills the clusters found by segmentation service
    {
        ROS_ERROR("ControlSequence - Segmentation error");
        state = BAD;
        good = false;
        return good;
    }
    state = INIT;
    return good;
}

void ControlSequence::start()
{
    visualization_msgs::MarkerArray tube_mrkr;

    if(!_clusters.empty())
    {
        _generate_tube_model(0);
        _tube->getCylinderMarker(tube_mrkr);
        _tube_mrkr_pub.publish(tube_mrkr);

        ROS_INFO("Getting Grasps");
        if(!_get_grasps())
        {
            ROS_ERROR("ControlSequence - Error in computing grasp");
            return;
        }

        if(_pick_up_tube("left_arm"))
        {
            ROS_INFO("Should be at approach position with tube grasped by right arm");
        }
        else if(_pick_up_tube("right_arm"))
        {
            ROS_WARN("ControlSequence - Pick up failed with right arm. trying with left hand...");
            ROS_INFO("Should be at approach position with tube grasped by left arm");
        }
        else
        {
            ROS_ERROR("ControlSequence - Error in picking tube");
            return;
        }
    }
    else
    {
        ROS_ERROR("ControlSequence - No cluster to generate tube model");
        return;
    }
    state = GOOD;
    return;
}

bool ControlSequence::_set_cloud_capture_posture()
{
    bool is_set = true;

    Gripper gripper;
    gripper.openRightGripper();
    gripper.openLeftGripper();
    ros::Duration(5).sleep();

    geometry_msgs::Pose pose;
    pose.position.x = 0.1;
    pose.position.y = -0.6;
    pose.position.z = 0.8;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    if(!_arms->moveRightArm(pose))
    {
        is_set = false;
        ROS_ERROR("ControlSequence - Unable to move right arm.");
    }

    pose.position.x = 0.1;
    pose.position.y = 0.6;
    pose.position.z = 0.8;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    if(!_arms->moveLeftArm(pose))
    {
        is_set = false;
        ROS_ERROR("ControlSequence - Unable to move left arm.");
    }

    if(!_pr2_head.lookAt(0.75,0.0,0.5))
    {
        is_set = false;
        ROS_ERROR("ControlSequence - Unable to set head to point.");
    }

    return is_set;
}


//calls tabletop segmentation service
bool ControlSequence::_get_segmented_cloud()
{
    tabletop_object_detector::TabletopSegmentation seg_srv;

    int MAX_TRY = 5;
    int cnt = 0;
    while(cnt<MAX_TRY)
    {
        cnt++;
        if(_seg_srv_client.call(seg_srv))
        {
            if(seg_srv.response.result == 3) // if seg_service fails to convert cloud frame in 3 attempts
                continue;
            cnt = MAX_TRY;
            if(seg_srv.response.result == seg_srv.response.SUCCESS)
            {
                sensor_msgs::PointCloud2 pc2;
                for(size_t i=0; i<seg_srv.response.clusters.size(); i++)
                {
                    sensor_msgs::convertPointCloudToPointCloud2(seg_srv.response.clusters[i], pc2);
                    _clusters.push_back(pc2);
                }
            }
            else
            {
                ROS_ERROR("ControlSequence - Segmentation service returned error %d", seg_srv.response.result);
                return false;
            }
        }
        else
        {
            ROS_ERROR("ControlSequence - Call to segmentation service failed");
            return false;
        }
    }
    if(_seg_srv_client.call(seg_srv))
    {
        if(seg_srv.response.result == seg_srv.response.SUCCESS)
        {
            sensor_msgs::PointCloud2 pc2;
            for(size_t i=0; i<seg_srv.response.clusters.size(); i++)
            {
                sensor_msgs::convertPointCloudToPointCloud2(seg_srv.response.clusters[i], pc2);
                _clusters.push_back(pc2);
            }
        }
        else
        {
            ROS_ERROR("ControlSequence - Segmentation service returned error %d", seg_srv.response.result);
            return false;
        }
    }
    else
    {
        ROS_ERROR("ControlSequence - Call to segmentation service failed");
        return false;
    }
    ROS_INFO("ControlSequence - In total, %d clusters found by segmentation service",_clusters.size());
    return true;
}

bool ControlSequence::_generate_tube_model(unsigned int cluster_idx)
{
    ROS_ASSERT_MSG(_clusters.size()>cluster_idx,"ControlSequence - Invalid cluster index");

    ROS_INFO("ControlSequence - Generating tube model using cluster no. %d", cluster_idx);
    _cloud_process.reset(new TubePerception::CloudProcessing(_clusters[cluster_idx], _tube));
    _cloud_process->setZerror(0);
    _cloud_process->processCloud();
}


bool ControlSequence::_get_grasps()
{
    _grasp_analysis.reset(new TubeGrasp::GraspAnalysis(_tube, _nh));
    geometry_msgs::Pose work_pose;
    work_pose.position.x = 0.7;
    work_pose.position.y = 0.0;
    work_pose.position.z = 0.8;
    work_pose.orientation.x = 0.0;
    work_pose.orientation.y = 0.0;
    work_pose.orientation.z = 0.0;
    work_pose.orientation.w = 1.0;
    _grasp_analysis->setWorkPose(work_pose);
    _grasp_analysis->analyze();
    if(_grasp_analysis->getComputedGraspPair(_grasp_pair))
    {
        _pick_pose = _grasp_analysis->getPickUpPose();
        return true;
    }
    else
        return false;
}

geometry_msgs::Pose move_in_X(geometry_msgs::Pose &pose_in, double dist)
{
    tf::Transform in, out, t;
    in = pose2tf(pose_in);
    t.setIdentity();
    t.setOrigin(tf::Vector3(dist, 0, 0));
    out = in*t;
    return tf2pose(out);
}

bool ControlSequence::_pick_up_tube(const std::string byWichArm)
{
    Gripper gripper;
    bool is_right_arm;

    if(byWichArm.compare("right_arm")==0)
        is_right_arm = true;
    else if(byWichArm.compare("left_arm")==0)
        is_right_arm = false;
    else
    {
        ROS_ERROR("ControlSequence - Illegal argument for _pick_up_tube()");
        return false;
    }

    /*if(is_right_arm)
        att_obj = _tube->getAttachedObjForRightGrasp(_pick_pose);
    else
        att_obj = _tube->getAttachedObjForLeftGrasp(_pick_pose);*/

     // approx 7 cm back from grasp pose
    geometry_msgs::Pose aprch_pose = move_in_X(_pick_pose, -0.08);

    if(is_right_arm)
    {
        gripper.openRightGripper();
        if(!_arms->moveRightArmWithMPlanning(aprch_pose))
        {
            ROS_ERROR("ControlSequence - Couldn't move right arm to approach position for pick up");
            return false;
        }
        state = AT_APRCH;
        if(!_arms->simpleMoveRightArm(_pick_pose))
        {
            ROS_ERROR("ControlSequence - Couldn't move right arm to pick up grasp position");
            return false;
        }
        state = AT_PICKUP;
        //TODO:  index of _tube->cylinders[]
        gripper.setRightGripperPosition(_tube->cylinders[0].radius*1.95,-1);
        ros::Duration(10).sleep();
        if(!_arms->simpleMoveRightArm(aprch_pose))
        {
            ROS_ERROR("ControlSequence - Couldn't move right arm back to approach position after pick up");
            return false;
        }
        state = GRASPED;
    }
    else
    {
        gripper.openLeftGripper();
        if(!_arms->moveRightArmWithMPlanning(aprch_pose))
        {
            ROS_ERROR("ControlSequence - Couldn't move left arm to approach position for pick up");
            return false;
        }
        state = AT_APRCH;
        if(!_arms->simpleMoveLeftArm(_pick_pose))
        {
            ROS_ERROR("ControlSequence - Couldn't move left arm to pick up grasp position");
            return false;
        }
        state = AT_PICKUP;
        //TODO:  index of _tube->cylinders[]
        gripper.setLeftGripperPosition(_tube->cylinders[0].radius*1.95,-1);
        ros::Duration(10).sleep();
        if(!_arms->simpleMoveLeftArm(aprch_pose))
        {
            ROS_ERROR("ControlSequence - Couldn't move left arm back to approach position after pick up");
            return false;
        }
        state = GRASPED;
    }
    return true;
}




