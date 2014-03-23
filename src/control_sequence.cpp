#include "controlSequence.h"


ControlSequence::ControlSequence(ros::NodeHandlePtr nh)
{
    _nh = nh;
    _seg_srv_client = _nh->serviceClient<tabletop_object_detector::TabletopSegmentation>("/tabletop_segmentation");

    _tube.reset(new TubePerception::Tube);
    _tube_mrkr_pub = _nh->advertise<visualization_msgs::MarkerArray>("/tube_polishing/tube_marker", 2);
    _collosion_obj_pub = _nh->advertise<arm_navigation_msgs::CollisionObject>("collision_object",10);
    if(!ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME,5))
        ROS_WARN_STREAM("Con not find "<<SET_PLANNING_SCENE_DIFF_NAME<<" service");
    _set_pln_scn = _nh->serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
    _att_obj.object.shapes.clear();
    _set_planning_scene();
    _arms.reset(new TubeManipulation::Arms(_nh));

    _attached_to_right_arm = false;
    _attached_to_left_arm = false;

    //_cloud_process.reset(new TubePerception::CloudProcessing());
    //_grasp_analysis.reset(new TubeGrasp::GraspAnalysis);

    _right_arm_home_jnts.resize(7);
    _right_arm_home_jnts[0] = -1.49;
    _right_arm_home_jnts[1] =  0.10;
    _right_arm_home_jnts[2] = -1.33;
    _right_arm_home_jnts[3] = -1.47;
    _right_arm_home_jnts[4] = -1.47;
    _right_arm_home_jnts[5] = -0.23;
    _right_arm_home_jnts[6] = -2.95;

    _left_arm_home_jnts.resize(7);
    _left_arm_home_jnts[0] =  1.49;
    _left_arm_home_jnts[1] =  0.10;
    _left_arm_home_jnts[2] =  1.33;
    _left_arm_home_jnts[3] = -1.47;
    _left_arm_home_jnts[4] =  1.47;
    _left_arm_home_jnts[5] = -0.23;
    _left_arm_home_jnts[6] = -2.95;
}

ControlSequence::~ControlSequence()
{
    ;
}

bool ControlSequence::initialize()
{
    if(!_set_cloud_capture_posture()){
        ROS_ERROR("ControlSequence - Unable to intitialize capture posture");
        return false;
    }

    _clusters.clear();
    if(!_get_segmented_cloud()){    //fills the clusters found by segmentation service
        ROS_ERROR("ControlSequence - Segmentation error");
        return false;
    }
    ROS_INFO("Initialized");
    return true;
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
        if(!_get_grasps()){
            ROS_ERROR("ControlSequence - Error in computing grasp");
            return;
        }

        if(_pick_up_tube("right_arm")){
            _get_attached_object();
            if(!_repos_tube_and_regrasp()){
                ROS_ERROR("ControlSequence - Error in regrasping");
                return;
            }
            else{
                bool r = _attached_to_right_arm, l = _attached_to_left_arm;
                /*_attached_to_right_arm = true;
                _attached_to_left_arm = true;*/
                _get_attached_object();
                _set_planning_scene();
                /*_attached_to_right_arm = r;
                _attached_to_left_arm = l;*/
                _move_arm_to_home_position("right_arm");
            }
            _get_trajectory();
            _move_to_staging_point();
        }
        else if( !_attached_to_right_arm )
        {
            geometry_msgs::Pose pose;
            pose.position.x = 0.1;
            pose.position.y = -0.6;
            pose.position.z = 0.8;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0;
            if(!_arms->moveRightArmWithMPlanning(pose)){
                return;
            }
            if(_pick_up_tube("left_arm")){
                ROS_WARN("ControlSequence - Pick up failed with right arm. trying with left arm...");

                _get_attached_object();
                if(!_repos_tube_and_regrasp()){
                    ROS_ERROR("ControlSequence - Error in regrasping");
                    return;
                }
            }
            else{
                ROS_ERROR("ControlSequence - Error in picking tube. couldn't reach by any arm");
                return;
            }
        }
        else{
            ROS_ERROR("ControlSequence - Error in picking tube");
            return;
        }
    }
    else{
        ROS_ERROR("ControlSequence - No cluster to generate tube model");
        return;
    }
    return;
}

bool ControlSequence::_set_cloud_capture_posture()
{
    ROS_INFO("Setting posture to cloud capture...");
    bool is_set = true;

    Gripper gripper;
    gripper.openRightGripper();
    gripper.openLeftGripper();
    //ros::Duration(5).sleep();

    if(!_move_arm_to_home_position("right_arm")){
        is_set = false;
    }
    if(!_move_arm_to_home_position("left_arm")){
        is_set = false;
    }
    if(!_pr2_head.lookAt(0.75,0.0,0.5)){
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
            else
            {
                cnt = MAX_TRY;
                if(seg_srv.response.result == seg_srv.response.SUCCESS)
                {
                    sensor_msgs::PointCloud2 pc2;
                    for(size_t i=0; i<seg_srv.response.clusters.size(); i++)
                    {
                        sensor_msgs::convertPointCloudToPointCloud2(seg_srv.response.clusters[i], pc2);
                        _clusters.push_back(pc2);
                    }
                    _get_table_as_object(seg_srv);
                    _collosion_obj_pub.publish(_table);

                }
                else
                {
                    ROS_ERROR("ControlSequence - Segmentation service returned error %d", seg_srv.response.result);
                    return false;
                }
            }
        }
        else
        {
            ROS_ERROR("ControlSequence - Call to segmentation service failed");
            return false;
        }
    }
    ROS_INFO("ControlSequence - In total, %d clusters found by segmentation service",_clusters.size());
    return true;
}

void ControlSequence::_get_table_as_object(tabletop_object_detector::TabletopSegmentation &seg_srv)
{
    double x_min = seg_srv.response.table.x_min,
           x_max = seg_srv.response.table.x_max,
           y_min = seg_srv.response.table.y_min,
           y_max = seg_srv.response.table.y_max;

    y_max = std::max(std::abs(y_min), std::abs(y_max));
    y_min = y_max * (-1);

    _table.header.frame_id = "base_link";
    _table.header.stamp = ros::Time::now();
    _table.id = "Table";
    _table.operation.operation = _table.operation.ADD;

    geometry_msgs::Pose pose;
    pose.orientation = seg_srv.response.table.pose.pose.orientation;
    pose.position.x = (x_min + x_max)/2;
    pose.position.y = (y_min + y_max)/2;
    pose.position.z = seg_srv.response.table.pose.pose.position.z;
    _table.poses.push_back(pose);

    arm_navigation_msgs::Shape box;
    box.type = box.BOX;
    box.dimensions.resize(3);
    box.dimensions[0] = (x_max - x_min);
    box.dimensions[1] = (y_max - y_min);
    box.dimensions[2] = 0.025;

    _table.shapes.push_back(box);
}

bool ControlSequence::_generate_tube_model(unsigned int cluster_idx)
{
    ROS_ASSERT_MSG(_clusters.size()>cluster_idx,"ControlSequence - Invalid cluster index");

    ROS_INFO("ControlSequence - Generating tube model using cluster no. %d", cluster_idx);
    _cloud_process.reset(new TubePerception::CloudProcessing);
    _cloud_process->setZerror(0);
    _cloud_process->genTubeModel(_clusters[cluster_idx], _tube);
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
        std::vector<tf::Vector3> points_to_avoid;
        geometry_msgs::Pose pose = _grasp_pair.leftGrasp.getGlobalPose(_tube->getTransform());
        tf::Vector3 vec(pose.position.x, pose.position.y, pose.position.z);
        points_to_avoid.push_back(vec);

        pose = _grasp_pair.rightGrasp.getGlobalPose(_tube->getTransform());
        vec.setValue(pose.position.x, pose.position.y, pose.position.z);
        points_to_avoid.push_back(vec);
        _pick_pose = _grasp_analysis->getPickUpPose(points_to_avoid, 0.1);
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

bool ControlSequence::_pick_up_tube(const std::string byWhichArm)
{
    Gripper gripper;
    bool is_right_arm;

    if(byWhichArm.compare("right_arm")==0)
        is_right_arm = true;
    else if(byWhichArm.compare("left_arm")==0)
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
        ROS_WARN("bool ControlSequence::_pick_up_tube(const std::string byWichArm) is Commentedout for testing");

//        gripper.openRightGripper();
//        ROS_INFO("Moving to approach position");
//        if(!_arms->moveRightArmWithMPlanning(aprch_pose))
//        {
//            ROS_ERROR("ControlSequence - Couldn't move right arm to approach position for pick up");
//            return false;
//        }

        //******************TESTING*****************//
        ROS_INFO("moving arm to pick position");
        if(!_arms->moveRightArmWithMPlanning(aprch_pose))
        {
            ROS_ERROR("ControlSequence - Couldn't move right arm to pick up grasp position");
            return false;
        }
        gripper.setRightGripperPosition(_tube->cylinders[0].radius*1.95,-1);
        _attached_to_right_arm = true;
        tf::Transform tube_tf = _tube->getTransform(),
                      pick_tf = pose2tf(_pick_pose), grasp_tf = tube_tf.inverseTimes(pick_tf);
        _current_grasp.rightGrasp.wristPose = tf2pose(grasp_tf);
        geometry_msgs::Pose wrist_pose = _arms->getRightArmFK();
        _tube->resetActualPose(_current_grasp.rightGrasp.wristPose, wrist_pose);
        //******************TESTING*****************//

//        ROS_INFO("moving arm to pick position");
//        if(!_arms->simpleMoveRightArm(_pick_pose))
//        {
//            ROS_ERROR("ControlSequence - Couldn't move right arm to pick up grasp position");
//            return false;
//        }

//        ROS_INFO("Closing gripper");
//        //TODO:  index of _tube->cylinders[]
//        gripper.setRightGripperPosition(_tube->cylinders[0].radius*1.95,-1);
//        _attached_to_right_arm = true;
//        ros::Duration(12).sleep();
//        aprch_pose = move_in_X(_pick_pose, -0.15);
//        if(!_arms->simpleMoveRightArm(aprch_pose)) //now aprch_pose is -0.15 m back
//        {
//            ROS_ERROR("ControlSequence - Couldn't move right arm back to approach position after pick up");
//            return false;
//        }
//        tf::Transform tube_tf = _tube->getTransform(),
//                      pick_tf = pose2tf(_pick_pose), grasp_tf = tube_tf.inverseTimes(pick_tf);

//        _current_grasp.rightGrasp.wristPose = tf2pose(grasp_tf);
//        geometry_msgs::Pose wrist_pose = _arms->getRightArmFK();
//        _tube->resetActualPose(_current_grasp.rightGrasp.wristPose, wrist_pose);
    }
    else
    {
        gripper.openLeftGripper();
        if(!_arms->moveLeftArmWithMPlanning(aprch_pose))
        {
            ROS_ERROR("ControlSequence - Couldn't move left arm to approach position for pick up");
            return false;
        }
        if(!_arms->simpleMoveLeftArm(_pick_pose))
        {
            ROS_ERROR("ControlSequence - Couldn't move left arm to pick up grasp position");
            return false;
        }
        //TODO:  index of _tube->cylinders[]
        gripper.setLeftGripperPosition(_tube->cylinders[0].radius*1.95,-1);
        _attached_to_left_arm = true;
        ros::Duration(12).sleep();
        if(!_arms->simpleMoveLeftArm(aprch_pose))
        {
            ROS_ERROR("ControlSequence - Couldn't move left arm back to approach position after pick up");
            return false;
        }
        tf::Transform tube_tf = _tube->getTransform(),
                      pick_tf = pose2tf(_pick_pose), grasp_tf = tube_tf.inverseTimes(pick_tf);

        _current_grasp.leftGrasp.wristPose = tf2pose(grasp_tf);
        geometry_msgs::Pose wrist_pose = _arms->getLeftArmFK();
        _tube->resetActualPose(_current_grasp.leftGrasp.wristPose, wrist_pose);
    }
    return true;
}

void ControlSequence::_get_attached_object()
{
    //arm_navigation_msgs::AttachedCollisionObject att_obj;
    if(_attached_to_right_arm && !_attached_to_left_arm)
        _att_obj = _tube->getAttachedObjForRightGrasp(_current_grasp.rightGrasp.wristPose);
    else if(!_attached_to_right_arm && _attached_to_left_arm)
        _att_obj = _tube->getAttachedObjForLeftGrasp(_current_grasp.leftGrasp.wristPose);
    else if(_attached_to_right_arm && _attached_to_left_arm)
        _att_obj = _tube->getAttachedObjForBothGrasps(_current_grasp.rightGrasp.wristPose);
    else
    {
        ROS_WARN("ControlSequence - attachedCollisionObject requested for tube! But tube is not grasped...");
        ROS_WARN("ControlSequence - ...by any arm. Setting object to empty");
        _att_obj.link_name.clear();
        _att_obj.object.poses.clear();
        _att_obj.object.shapes.clear();
    }
    _set_planning_scene();
}

void ControlSequence::_set_planning_scene()
{
    arm_navigation_msgs::SetPlanningSceneDiff::Request req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response res;
    if(!_att_obj.object.shapes.empty()){
        req.planning_scene_diff.attached_collision_objects.push_back(_att_obj);
    }
    if(!_set_pln_scn.call(req,res)){
        ROS_ERROR("Couldn't set planning scene");
    }
}

bool ControlSequence::_repos_tube_and_regrasp()
{
    _set_planning_scene();
    geometry_msgs::Pose tube_pose_out, new_wrist_pose;
    if(_attached_to_right_arm && !_attached_to_left_arm)
    {
        std::vector<double> left_ik_joints;
        _arms->getRegraspPoseRight(_att_obj, _current_grasp.rightGrasp.wristPose,
                                   _arms->getRightArmFK(),
                                   _grasp_pair.leftGrasp.wristPose,
                                   tube_pose_out,
                                   left_ik_joints);
        tf::Transform tube = pose2tf(tube_pose_out), wrist;
        wrist = pose2tf(_current_grasp.rightGrasp.wristPose);
        wrist = tube * wrist;
        new_wrist_pose = tf2pose(wrist);
        if(!_arms->moveRightArmWithMPlanning(new_wrist_pose))
        {
            ROS_ERROR("ControlSequence - Failed to move right arm for regrasp pose.");
            return false;
        }
        Gripper gripper;
        gripper.openLeftGripper();
        geometry_msgs::Pose aprch_pose = move_in_X(_grasp_pair.leftGrasp.wristPose, -0.1);
        _tube->resetActualPose(_current_grasp.rightGrasp.wristPose,new_wrist_pose);
        _get_attached_object();
        /*if(!_arms->moveLeftArmWithMPlanning(aprch_pose))
        {
            ROS_ERROR("ControlSequence - Failed to move left arm to approach position for regrasping");
            return false;
        }*/
        if(!_arms->moveLeftArmWithMPlanning(left_ik_joints)){
            ROS_ERROR("ControlSequence - Failed to move left arm to grasp position regrasping");
            return false;
        }
        gripper.setLeftGripperPosition(_tube->cylinders[0].radius*1.95,-1);
        ros::Duration(5).sleep();
        _attached_to_left_arm = true;
        gripper.openRightGripper();
        ros::Duration(5).sleep();
        _attached_to_right_arm = false;
        _current_grasp.leftGrasp = _grasp_pair.leftGrasp;
        _set_planning_scene();
        return true;
    }
    else if(!_attached_to_right_arm && _attached_to_left_arm)
    {
        std::vector<double> right_ik_joints;
        _arms->getRegraspPoseLeft(_att_obj, _current_grasp.leftGrasp.wristPose,
                                  _arms->getLeftArmFK(),
                                  _grasp_pair.rightGrasp.wristPose,
                                  tube_pose_out,
                                  right_ik_joints);
        tf::Transform tube = pose2tf(tube_pose_out), wrist;
        wrist = pose2tf(_current_grasp.leftGrasp.wristPose);
        wrist = tube * wrist;
        new_wrist_pose = tf2pose(wrist);
        if(!_arms->moveLeftArmWithMPlanning(new_wrist_pose))
        {
            ROS_ERROR("ControlSequence - Failed to move left arm for regrasp pose.");
            return false;
        }
        Gripper gripper;
        gripper.openRightGripper();
        geometry_msgs::Pose aprch_pose = move_in_X(_grasp_pair.rightGrasp.wristPose, -0.1);
        _tube->resetActualPose(_current_grasp.leftGrasp.wristPose,new_wrist_pose);
        _get_attached_object();
        /*if(!_arms->moveRightArmWithMPlanning(aprch_pose))
        {
            ROS_ERROR("ControlSequence - Failed to move right arm to approach position for regrasping");
            return false;
        }*/
        if(!_arms->simpleMoveRightArm(_grasp_pair.rightGrasp.wristPose))
        {
            ROS_ERROR("ControlSequence - Failed to move right arm to grasp position regrasping");
            return false;
        }
        gripper.setRightGripperPosition(_tube->cylinders[0].radius*1.95,-1);
        ros::Duration(5).sleep();
        _attached_to_right_arm = true;
        gripper.openLeftGripper();
        ros::Duration(5).sleep();
        _attached_to_left_arm = false;
        _current_grasp.rightGrasp = _grasp_pair.rightGrasp;
        return true;
    }
    else
    {
        ROS_ERROR("ControlSequence - Error regrasping. Both or none is/are grasping.");
        return false;
    }
}


void ControlSequence::_get_trajectory()
{
    bool attached_to_right = _attached_to_right_arm,
            attached_to_left = _attached_to_left_arm;
    geometry_msgs::PoseArray tube_traj;
    _grasp_analysis->getTubeWorkTrajectory(tube_traj);
    std::vector<double> right_traj, left_traj;
    arm_navigation_msgs::AttachedCollisionObject att_obj = _att_obj;
    _att_obj = _tube->getAttachedObjForBothGrasps(_grasp_pair.rightGrasp.wristPose);
    _attached_to_right_arm = true;
    _attached_to_left_arm = true;
    _set_planning_scene();
    _arms->genTrajectory(_att_obj, tube_traj,_grasp_pair.rightGrasp.wristPose, _grasp_pair.leftGrasp.wristPose, right_traj, left_traj);
    _arms->setTrajectory(right_traj, left_traj);
    //as it was
    _att_obj = att_obj;
    _attached_to_right_arm = attached_to_right;
    _attached_to_left_arm = attached_to_left;
    _set_planning_scene();
}

bool ControlSequence::_move_arm_to_home_position(std::string which_arm)
{
    ROS_INFO_STREAM("Moving "<<which_arm<<" to home position...");
    bool right_arm = false,
         left_arm = false;


    if(which_arm.compare("right_arm")==0)
        right_arm = true;
    else if(which_arm.compare("left_arm")==0)
        left_arm = true;
    else if(which_arm.compare("both_arms")==0)
    {
        right_arm = true;
        left_arm = true;
    }
    else
    {
        ROS_ERROR("ControlSequence - Illegal argument for_move_arm_to_home_pos(std::string which_arm)");
        return false;
    }

    if(right_arm)
    {
        if(!_arms->moveRightArmWithMPlanning(_right_arm_home_jnts))
        {
            ROS_INFO("ControlSequence - failed to move right arm to home position");
            return false;
        }
    }
    if(left_arm)
    {
        if(!_arms->moveLeftArmWithMPlanning(_left_arm_home_jnts))
        {
            ROS_INFO("ControlSequence - failed to move left arm to home position");
            return false;
        }
    }
    ROS_DEBUG("Finished moving arm(s) to home position");
    return true;
}

bool ControlSequence::_move_to_staging_point()
{
    _set_planning_scene();
    if(_attached_to_right_arm && !_attached_to_left_arm)
    {
        ROS_INFO("ControlSequence - Moving Right arm to first trajectory point");
        if(_grasp_pair.qRight.size()>=7 && _grasp_pair.qLeft.size()>=7)
        {
            std::vector<double> right_joints(7), left_joints(7);
            for(size_t i=0; i<7; i++) //first point i.e. first 7 joint values
            {
                right_joints[i] = _grasp_pair.qRight[i];
                left_joints[i] = _grasp_pair.qLeft[i];
            }

            geometry_msgs::Pose pose = _arms->getRightArmFK(right_joints);
            _arms->moveRightArmWithMPlanning(pose);
            pose = _arms->getLeftArmFK(left_joints);
            geometry_msgs::Pose aprch = move_in_X(pose, -0.1);
            _arms->moveLeftArmWithMPlanning(aprch);
            /*_arms->genTrajectory();
            _arms->executeJointTrajectory();*/
            return true;
        }
        else
        {
            ROS_WARN("less than one point in trajectory");
            return false;
        }
    }
    else if(!_attached_to_right_arm && _attached_to_left_arm)
    {
        if(!_move_arm_to_home_position("right_arm")){
            return false;
        }
        ROS_INFO("ControlSequence - Moving Left arm to first trajectory point");
        if(_grasp_pair.qRight.size()>=7 && _grasp_pair.qLeft.size()>=7)
        {
            std::vector<double> right_joints(7), left_joints(7);
            for(size_t i=0; i<7; i++) //first point i.e. first 7 joint values
            {
                right_joints[i] = _grasp_pair.qRight[i];
                left_joints[i] = _grasp_pair.qLeft[i];
            }
            geometry_msgs::Pose pose = _arms->getRightArmFK(right_joints);
            _arms->moveLeftArmWithMPlanning(left_joints);
            pose = _arms->getLeftArmFK(left_joints);
            //geometry_msgs::Pose aprch = move_in_X(pose, -0.1);
            _arms->moveRightArmWithMPlanning(right_joints);
            /*_arms->genTrajectory();
            _arms->executeJointTrajectory();*/
            return true;
        }
        else
        {
            ROS_WARN("less than one point in trajectory");
            return false;
        }
    }
    else
    {
        ROS_WARN("ControlSequence - tube is not attached to any arm");
        return false;
    }
}
