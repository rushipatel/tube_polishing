#include "state_machine.h"

#define LGRNM "stateMachine"

stateMachine::stateMachine(ros::NodeHandlePtr nh){
    _nh = nh;
    _att2right = false;
    _att2left = false;
    _state = INIT;
    _MAX_REGRASP_TRY = 3;
    _regrasp_try = 0;
    _cluster_idx = 0;
    _work_traj_idx = 0;
    _r_soln_avail = false;
    _l_soln_avail = false;
    _WRIST_OFFSET = 0.15;
    _PICK_WRIST_OFFSET = 0.19;
    _TABLE_HEIGHT = 0.5; //in meters
    _z_error = 0.0;

    _table_obj_id = "Table";
    _att_obj_id = "Tube";
    _tube_obj_id = "staticTube";

    _att_obj.reset(new arm_navigation_msgs::AttachedCollisionObject);
    _collision_objects.reset(new collisionObjects(_nh));
    _collision_objects->setPlanningScene();

    _gripper.reset(new Gripper(_nh));
    _arms.reset(new TubeManipulation::Arms(_nh,_collision_objects));
    //_arms->setAttachedObjPtr(_att_obj);
    _cloud_process.reset(new TubePerception::CloudProcessing);
    _grasp_analysis.reset(new TubeGrasp::GraspAnalysis(_nh,_collision_objects));
    _collision_check.reset(new TubeManipulation::CollisionCheck(_nh, _collision_objects));
    _tube.reset(new TubePerception::Tube);
    _seg_srv_client = _nh->serviceClient
            <tabletop_object_detector::TabletopSegmentation>
            ("/tabletop_segmentation");

    _tube_mrkr_pub = _nh->advertise<visualization_msgs::MarkerArray>("/tube_polishing/tube_marker", 2);
    //_collision_obj_pub = _nh->advertise<arm_navigation_msgs::CollisionObject>("collision_object", 2);
    _grasp_mrkr_pub = _nh->advertise<visualization_msgs::MarkerArray>("/tube_polishing/grasp_marker", 2);
    _pick_grasp_pub = _nh->advertise<geometry_msgs::PoseStamped>("/tube_polishing/lift_grasp_pose", 2);
    _work_point_pub = _nh->advertise<visualization_msgs::MarkerArray>("/tube_polishing/work_points_marker",2);

    /*if(!ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME,5))
        ROS_WARN_STREAM("Can not find "<<SET_PLANNING_SCENE_DIFF_NAME<<" service");
    _set_pln_scn = _nh->serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);*/


    //set values for home position of arms
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

    _work_pose1.position.x = 0.7;
    _work_pose1.position.y = 0.0;
    _work_pose1.position.z = 0.8;
    _work_pose1.orientation.x = 0.0;
    _work_pose1.orientation.y = 0.0;
    _work_pose1.orientation.z = 0.0;
    _work_pose1.orientation.w = 1.0;

    _work_pose2 = _work_pose1;
}

void stateMachine::start()
{
    while(ros::ok())
    {
        switch(_state){

        //move arms to home position and point head towards table. get ready to capture point cloud
        case INIT:
        {
            ROS_INFO_NAMED(LGRNM,"*Initializing...");
            _update_scene();
            _gripper->openRightGripper();
            _gripper->openLeftGripper();
            if(!_move_arm_to_home_position("both_arms")){
                _state = ERR;
                break;
            }
            _head.lookAt(0.75,0.0,0.5);
            _state = PERCIEVE;
            //_state = DONE;
            ROS_INFO_NAMED(LGRNM,"*Initialized");
            break;
        }
        case PERCIEVE:
        {
            ROS_INFO_NAMED(LGRNM,"*Percieving...");
            _update_scene();
            if(!_get_clusters()){ //gets clusters on table and publishes table as collision object
                _state = ERR;
                break;
            }
            if(!_gen_tube_model()){
                _state = ERR;
                break;
            }
            /*sensor_msgs::PointCloud2ConstPtr cloud_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>
                    ("/head_mount_kinect/depth_registered/points");
            sensor_msgs::PointCloud2::Ptr cloud_filtered(new sensor_msgs::PointCloud2);
            _cloud_process->extractTubePoints(_tube, cloud_ptr, cloud_filtered);
            _state = DONE;
            break;*/
            _update_scene();
            _publish_tube();
            visualization_msgs::MarkerArray marker_array;
            _tube->getWorkPointsMarker(marker_array);
            marker_array.markers.clear();
            _work_point_pub.publish(marker_array);
            //_add_tube_to_collision_space();
            _update_scene();
            //TODO: get_attached_obj;
            _state = GRASP_ANLYS;
            ROS_INFO_NAMED(LGRNM,"*Generated tube and published table");
            break;
        }
        case GRASP_ANLYS:
        {
            ROS_INFO_NAMED(LGRNM,"*Analyzing Grasps...");
            _update_scene();
            if(!_get_computed_grasp_pair()){
                _state = ERR;
                break;
            }
            _publish_grasps();
            if(!_get_pick_grasp()){
                ROS_ERROR_NAMED(LGRNM,"No solution of lift grasp found for any arm");
                _state = ERR;
                break;
            }
            _state = PICK;
            break;
        }
        case PICK:
        {
            ROS_INFO("*Picking tube...");
            _print_state();
            if(_r_soln_avail){
                ROS_INFO("Lifting with right arm");
                //if(_arms->moveRightArmWithMPlanning(_pick_grasp.getWristGlobalPose(_tube->getPose()))){
                    if(_lift_obj_with_right_arm()){
                        _l_soln_avail = false; //done with these variables as of now. reset them
                        _r_soln_avail = false;
                 //   }
                    break;
                }
                else{
                    //_r_soln_avail = false;
                    if(!_move_arm_to_home_position("right_arm")){
                        _state = ERR;
                    }
                    break; //leave with the state = PICK. this will try with left arm
                }
            }
            if(_l_soln_avail){
                /*if(_arms->moveLeftArmWithMPlanning(_pick_grasp.getWristGlobalPose(_tube->getPose()))){
                    //TODO: handle lift sequence;
                }
                else{
                    _l_soln_avail = false;
                    if(!_move_arm_to_home_position("left_arm")){
                        _state = ERR;
                    }
                    break;
                }*/
            }
            _state = REGRASP;
            break;
        }
        case REGRASP:
        {
            ROS_INFO("*Regrasping...");
            _current_left_grasp = _computed_grasp_pair.leftGrasp;
            if(!_regrasp()){
                _state = ERR;
                break;
            }
            _state = TRAJ_GEN;
            break;
        }
        case TRAJ_GEN:
        {
            ROS_INFO("*Generating trajectory...");
            _update_scene();
            if(_att2left){
                if(!_arms->moveLeftArmWithMPlanning(_computed_grasp_pair.qLeft)){ //first 7 values
                    _state = ERR;
                }
                _att2right = true;
                if(!_arms->moveRightArmWithMPlanning(_computed_grasp_pair.qRight)){
                    _state = ERR;
                }
                if(!_arms->executeJointTrajectory(_computed_grasp_pair.qRight, _computed_grasp_pair.qLeft)){
                    _state = ERR;
                }
            }
            _state = DONE;
            break;
        }
        case TRAJ_EXE:
        {
            ROS_INFO("*executing trajectory...");
            break;
        }
        case ERR:
        {
            _print_state();
            ROS_ERROR("*Error! Shutting down...");
            ros::shutdown();
            break;
        }
        case DONE:
        {
            ROS_INFO("*Done! Shutting down...");
            _print_state();
            ros::shutdown();
            break;
        }
        default:
        {
            _print_state();
            ROS_ERROR("Something went wrong with the state! Shutting down...");
            ros::shutdown();
        }
        }//switch
    }
}


void stateMachine::_update_scene(void){
    _get_attached_obj();
    _collision_objects->addAttachedCollisionObject(*_att_obj);
    _collision_objects->addCollisionObject(_table);

    _tube->getCollisionObject(_tube_collision_obj);
    if(!(_tube->cylinders.empty() || _att2right || _att2left)){
        //_tube_collision_obj.id = _tube_obj_id;
        geometry_msgs::Pose pose;
        arm_navigation_msgs::Shape shape;
        for(unsigned int i=0; i<_tube_collision_obj.poses.size(); i++){
            pose = _tube_collision_obj.poses[i];
            shape = _tube_collision_obj.shapes.at(i);
            //adjust the height of object
            if(pose.position.z<(_TABLE_HEIGHT+shape.dimensions.at(0))){
                ROS_WARN_NAMED(LGRNM,
                "z value of static_tube pose origin is less than height of table plus its radius. Adjusting z value");
                //_tube_collision_obj.poses[i].position.z = _TABLE_HEIGHT+shape.dimensions.at(0);
            }
        }
        _collision_objects->addCollisionObject(_tube_collision_obj);
    }
    else{
        _collision_objects->removeCollisionObject(_tube_collision_obj.id.c_str());
    }
    _collision_objects->setPlanningScene();
}

bool stateMachine::_move_arm_to_home_position(std::string which_arm)
{
    ROS_INFO_STREAM_NAMED(LGRNM,"Moving "<<which_arm<<" to home position...");
    bool right_arm = false, left_arm = false;
    if(which_arm.compare("right_arm")==0)
        right_arm = true;
    else if(which_arm.compare("left_arm")==0)
        left_arm = true;
    else if(which_arm.compare("both_arms")==0){
        right_arm = true;
        left_arm = true;
    }
    else{
        ROS_ERROR("ControlSequence - Illegal argument for_move_arm_to_home_pos(std::string which_arm)");
        return false;
    }

    if(right_arm){
        if(!_arms->moveRightArmWithMPlanning(_right_arm_home_jnts)){
            ROS_WARN_NAMED(LGRNM,"failed to move right arm to home position");
            return false;
        }
    }
    if(left_arm){
        if(!_arms->moveLeftArmWithMPlanning(_left_arm_home_jnts)){
            ROS_WARN_NAMED(LGRNM,"failed to move left arm to home position");
            return false;
        }
    }
    ROS_DEBUG_NAMED(LGRNM,"Finished moving %s to home position",which_arm.c_str());
    return true;
}
bool stateMachine::_get_clusters(void){
    ROS_INFO_NAMED(LGRNM,"Getting point clusters from tabletop_segmentation service...");
    tabletop_object_detector::TabletopSegmentation seg_srv;
    int MAX_TRY = 5; //segmentation service fails for few times.
    int cnt = 0;
    while(cnt<MAX_TRY){
        if(_seg_srv_client.call(seg_srv)){
            if(seg_srv.response.result == 3) // if seg_service fails to convert cloud frame in 3 attempts
                continue;
            else{
                cnt = MAX_TRY;
                if(seg_srv.response.result == seg_srv.response.SUCCESS){
                    _clusters.clear();
                    sensor_msgs::PointCloud2 pc2;
                    for(size_t i=0; i<seg_srv.response.clusters.size(); i++){
                        sensor_msgs::convertPointCloudToPointCloud2(seg_srv.response.clusters[i], pc2);
                        _clusters.push_back(pc2);
                    }
                    _extract_table_from_msg(seg_srv);
                }
                else{
                    ROS_ERROR_NAMED(LGRNM,"Segmentation service returned error %d", seg_srv.response.result);
                    return false;
                }
            }
        }
        else{
            ROS_ERROR_NAMED(LGRNM,"Call to segmentation service failed");
            return false;
        }
        cnt++;
    }
    ROS_INFO_NAMED(LGRNM,"segmentation service returned %d clusters",_clusters.size());
    return true;
}

//uses cloudProcessing class to generate model
bool stateMachine::_gen_tube_model(void){
    _tube->reset();
    _cloud_process->setZerror(_z_error);
    if(!_cloud_process->genTubeModel(_clusters.at(_cluster_idx),_tube)){
        return false;
    }
    ROS_INFO_NAMED(LGRNM,"Tube model generated with %d cylinders",_tube->cylinders.size());
    return true;
}

//get table parameter from response and publish it
void stateMachine::_extract_table_from_msg(tabletop_object_detector::TabletopSegmentation &seg_srv)
{
    double x_min = seg_srv.response.table.x_min,
           x_max = seg_srv.response.table.x_max,
           y_min = seg_srv.response.table.y_min,
           y_max = seg_srv.response.table.y_max;

    y_max = std::max(std::abs(y_min), std::abs(y_max));
    y_min = y_max * (-1);

    _table.header.frame_id = "/base_link";
    _table.header.stamp = ros::Time::now();
    _table.id = _table_obj_id;
    _table.operation.operation = _table.operation.ADD;

    geometry_msgs::Pose pose;
    pose.orientation = seg_srv.response.table.pose.pose.orientation;
    pose.position.x = (x_min + x_max)/2;
    pose.position.y = (y_min + y_max)/2;
    pose.position.z = seg_srv.response.table.pose.pose.position.z;
    _z_error = _TABLE_HEIGHT - pose.position.z;
    ROS_INFO_NAMED(LGRNM,"Table height is %f. Z error = %f", pose.position.z, _z_error);
    double box_height = 0.025;
    pose.position.z = _TABLE_HEIGHT;
    pose.position.z -= (box_height/2);

    _table.poses.push_back(pose);

    arm_navigation_msgs::Shape box;
    box.type = box.BOX;
    box.dimensions.resize(3);
    box.dimensions[0] = (x_max - x_min);
    box.dimensions[1] = (y_max - y_min);
    box.dimensions[2] = box_height;

    _table.shapes.push_back(box);

}

/*void stateMachine::_remove_table_from_collision_space(){
    _table.id = "Table";
    _table.operation.operation = _table.operation.REMOVE;
    _collision_obj_pub.publish(_table);
}

void stateMachine::_add_table_to_collision_space(){
    if(_table.shapes.empty()){
        return;
    }
    else{
        _collision_obj_pub.publish(_table);
    }
}

void stateMachine::_add_tube_to_collision_space(){
    if(!_tube->cylinders.empty()){
        _tube->getCollisionObject(_tube_collision_obj);
    }
    _collision_obj_pub.publish(_tube_collision_obj);
}

void stateMachine::_remove_tube_from_collision_space(){
    if(!_tube->cylinders.empty()){
        _tube->getCollisionObject(_tube_collision_obj);
    }
    _tube_collision_obj.operation.operation = _tube_collision_obj.operation.REMOVE;
    _collision_obj_pub.publish(_tube_collision_obj);
}*/

void stateMachine::_publish_tube(void){
    visualization_msgs::MarkerArray marker_array;
    _tube->getCylinderMarker(marker_array);
    _tube_mrkr_pub.publish(marker_array);
}

void stateMachine::_publish_grasps(void){
    visualization_msgs::MarkerArray marker_array;
    _grasp_analysis->getGraspMarker(_tube, _WRIST_OFFSET, marker_array);
    _grasp_mrkr_pub.publish(marker_array);
}

void stateMachine::_publish_pick_pose(void){
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "/base_link";
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose = _pick_grasp.getWristGlobalPose(_tube->getPose());
    _pick_grasp_pub.publish(pose_stamped);
}

bool stateMachine::_get_computed_grasp_pair()
{
    ROS_INFO_NAMED(LGRNM,"Computing for grasp pair...");
    _grasp_analysis->setTubePtr(_tube);
    _grasp_analysis->setWorkPose(_work_pose1);
    _grasp_analysis->setWorkTrajIdx(_work_traj_idx);
    _grasp_analysis->compute();
    if(!_grasp_analysis->getComputedGraspPair(_computed_grasp_pair)){
        ROS_WARN_NAMED(LGRNM,"No computed grasp pair is available!");
        return false;
    }
    ROS_INFO_NAMED(LGRNM,"Computed grasp pair found");
    return true;
}

bool stateMachine::_get_pick_grasp(void)
{
    _grasp_analysis->setTubePtr(_tube);
    std::vector<tf::Vector3> points;
    tf::Vector3 vec;
    tf::Transform tube_tf = _tube->getTransform(), wrist_tf;
    wrist_tf = _computed_grasp_pair.rightGrasp.getWristGlobalPose(tube_tf);
    vec = wrist_tf.getOrigin();
    points.push_back(vec);
    wrist_tf = _computed_grasp_pair.leftGrasp.getWristGlobalPose(tube_tf);
    vec = wrist_tf.getOrigin();
    points.push_back(vec); //points to avoid
    _pick_grasp = _grasp_analysis->getPickPose(points,0.1);
    _collision_check->refreshState();
    _collision_check->enableVisualization();
    _collision_check->setMarkerLifeTime(60);
    std::vector<double> right_jnts, left_jnts;

    _r_soln_avail = false;
    _l_soln_avail = false;
    _pick_grasp.setWristOffset(_PICK_WRIST_OFFSET);
    _publish_pick_pose();
    geometry_msgs::Pose pick_pose = _pick_grasp.getWristGlobalPose(_tube->getPose());
    //_update_scene();
    if(_arms->getSimpleRightArmIK(pick_pose, right_jnts)){
        //_arms->getLeftJoints(left_jnts);
        //if(_collision_check->isRightArmStateValid(right_jnts)){
            _r_soln_avail = true;
        //}
        //else{
        //    ROS_INFO_STREAM_NAMED(LGRNM,"Collision Check error : "<<_collision_check->getLastErrorAsString());
        //}
    }

    if(_arms->getSimpleLeftArmIK(pick_pose, left_jnts)){
        //_arms->getRightJoints(right_jnts);
        //if(_collision_check->isLeftArmStateValid(left_jnts)){
            _l_soln_avail = true;
        //}
        //else{
        //    ROS_INFO_STREAM_NAMED(LGRNM,"Collision Check error : "<<_collision_check->getLastErrorAsString());
        //}
    }
    _collision_check->disableVisualization();
    if(_r_soln_avail | _l_soln_avail){
        return true;
    }
    return false;
}

void stateMachine::_get_attached_obj(){
    if(_att2right && _att2left){
       _tube->getAttachedObjForBothGrasps(_current_right_grasp.getWristPose(), _att_obj);
    }
    else if(_att2right){
       _tube->getAttachedObjForRightGrasp(_current_right_grasp.getWristPose(), _att_obj);
    }
    else if(_att2left){
       _tube->getAttachedObjForLeftGrasp(_current_left_grasp.getWristPose(), _att_obj);
    }
    else{
        _att_obj->object.shapes.clear();
    }
    //_att_obj->object.id = _att_obj_id;
}

bool stateMachine::_lift_obj_with_right_arm(void)
{
    ROS_INFO_NAMED(LGRNM,"Lifting object with right arm...");
    _update_scene();
    _pick_grasp.setWristOffset(_PICK_WRIST_OFFSET+
                               (_tube->cylinders[_pick_grasp.cylinderIdx].radius*4)); //clearance is 3 times of radius
    geometry_msgs::Pose approach_pose = _pick_grasp.getWristGlobalPose(_tube->getPose());
    //_collision_objects->setAllowedContactCube(approach_pose,0.1);
    std::vector<double> ik_soln;
    if(!_arms->getRightArmIK(approach_pose, ik_soln)){
        ROS_WARN_NAMED(LGRNM,"no IK solution for appraoch pose");
        return false;
    }

    if(!_arms->moveRightArmWithMPlanning(ik_soln)){
        return false;
    }
    _pick_grasp.setWristOffset(_PICK_WRIST_OFFSET);
    geometry_msgs::Pose pick_pose =
            _pick_grasp.getWristGlobalPose(_tube->getPose());
    ROS_WARN_NAMED(LGRNM,
    "Pick pose Z value is a fixed value i.e. _TABLE_HEIGHT + _PICK_WRIST_OFFSET + 0.01");
    pick_pose.position.z = _TABLE_HEIGHT + _PICK_WRIST_OFFSET + 0.01;
    if(!_arms->simpleMoveRightArm(pick_pose)){
        return false;
    }
    if(!_gripper->setRightGripperPosition(_tube->cylinders[_pick_grasp.cylinderIdx].radius*1.5, -1)){
        return false;
    }
    _att2right = true;
    _current_right_grasp = _pick_grasp;
    if(!_arms->simpleMoveRightArm(approach_pose)){
        return false;
    }
    tf::Transform tube, wrist=pose2tf(approach_pose), grasp = pose2tf(_pick_grasp.getWristPose());
    tube = wrist * grasp.inverse();
    geometry_msgs::Pose new_tube_pose = tf2pose(tube);
    _tube->setPose(new_tube_pose);
    _tube->setPoseAsActualPose();
    _update_scene();
    ROS_INFO_NAMED(LGRNM,"Lifted object with right arm");
    return true;
}

bool stateMachine::_lift_obj_with_left_arm(void)
{
    return false;
}

// requires att2* only one bit set
// _current_right_grasp
// _current_left_grasp
bool stateMachine::_regrasp(){
    if(_att2right&&_att2left){
        ROS_WARN_NAMED(LGRNM,"Can not regrasp. Attached to both arms!");
        return false;
    }

    if(_att2right){
        geometry_msgs::Pose new_pose;  //tube's new pose
        std::vector<double> ik_soln;
        if(!_arms->getRegraspPoseRight(_att_obj,
                                       _current_right_grasp.getWristPose(),
                                       _arms->getRightArmFK(),
                                       _computed_grasp_pair.leftGrasp.getWristPose(),
                                       new_pose, ik_soln)){
            return false;
        }
        ROS_INFO_NAMED(LGRNM,"Trying to move arm to cloud capture pose...");
        tf::Transform tube_tf = _tube->getTransform();
        std::vector<double> ik_soln2(7);
        if(_arms->moveRightArmToCloudCapturePose(tf::Vector3(0,0,1.5),tube_tf.getOrigin(),1.0, ik_soln2)){
            if(_arms->moveRightArmWithMPlanning(ik_soln2)){
                ros::Duration(3).sleep();
                tf::Transform tube_tf, grsp = _current_right_grasp.getWristTransform(), fk = pose2tf(_arms->getRightArmFK(ik_soln2));
                tube_tf = fk*grsp.inverse();
                geometry_msgs::Pose tube_pose = tf2pose(tube_tf);
                _tube->setPose(tube_pose);
                _tube->setPoseAsActualPose();
                _publish_tube();
                //sensor_msgs::PointCloud2ConstPtr cloud_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>
                //                ("/wide_stereo/points2");
                sensor_msgs::PointCloud2ConstPtr cloud_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>
                        ("/head_mount_kinect/depth_registered/points",ros::Duration(5));
                sensor_msgs::PointCloud2::Ptr cloud_filtered(new sensor_msgs::PointCloud2);
                _cloud_process->extractTubePoints(_tube, cloud_ptr, cloud_filtered);
                //ros::Duration(10).sleep();
            }
            ROS_WARN_NAMED(LGRNM,"Couldn't move to cloud capture pose!");
        }
        else{
            ROS_WARN_NAMED(LGRNM, "Couldn't get cloud capture pose!");
        }
        _tube->setPose(new_pose);
        _tube->setPoseAsActualPose();
        new_pose = _current_right_grasp.getWristGlobalPose(new_pose);//new pose of wrist
        if(!_arms->moveRightArmWithMPlanning(new_pose)){
            return false;
        }
        ROS_INFO_NAMED(LGRNM,"Moved to new pose for regrasping");
        //_att2left = true;
        //_update_scene();
        //move left arm to _computed_grasp_pair.leftGrasp.wristPose
        if(!_arms->moveLeftArmWithMPlanning(ik_soln)){
            return false;
        }
        ROS_INFO_NAMED(LGRNM,"Moved left arm to new grasp pose");
        //close left gripper
        if(!_gripper->setLeftGripperPosition(_tube->cylinders[_pick_grasp.cylinderIdx].radius*1.9, -1)){
            return false;
        }
        if(!_gripper->openRightGripper()){
            return false;
        }
        _current_right_grasp.setWristOffset(_PICK_WRIST_OFFSET+(_tube->cylinders[_current_right_grasp.cylinderIdx].radius*4));
        geometry_msgs::Pose wrist_pose = _current_right_grasp.getWristGlobalPose(_tube->getPose());
        if(!_arms->moveRightArmWithMPlanning(wrist_pose)){
            return false;
            _att2left = true;
            _att2right = false;
        }
        _att2left = true;
        _att2right = false;
        _current_left_grasp =  _computed_grasp_pair.leftGrasp;
        _get_attached_obj();
        _update_scene();
        _move_arm_to_home_position("right_arm");
    }

    return true;
}

void stateMachine::_print_state(){
    std::cout<<"\n********************State variables***********************";
    std::cout<<"\n   att2right = "<<_att2right<<"\tatt2left = "<<_att2left;
    std::cout<<"\n   r_soln_avail = "<<_r_soln_avail<<"\tl_soln_avail = "<<_l_soln_avail;
    std::cout<<"\n   state = "<<_get_state_str(_state);
    std::cout<<"\n   number of cylinders in tube = "<<_tube->cylinders.size();
    std::cout<<"\n   number of work point clusters = "<<_tube->workPointsCluster.size();
    std::cout<<"\n   attached obj has "<<_att_obj->object.shapes.size()<<" shapes";
    std::cout<<"\n   att_obj_ptr use_count :" <<_att_obj.use_count();
    std::cout<<"\n   table as collision object has "<<_table.shapes.size()<<" shapes";
    std::cout<<"\n   number of clusters = "<<_clusters.size();
    _collision_objects->printListOfObjects();
    std::cout<<"\n**********************************************************\n";
}

std::string stateMachine::_get_state_str(int state){
    std::string str;
    if(state == INIT)
        str="INIT";
    else if(state == PERCIEVE)
        str="PERCIEVE";
    else if(state == PICK)
        str="PICK";
    else if(state == REGRASP)
        str="REGRASP";
    else if(state == TRAJ_GEN)
        str="TRAJ_GEN";
    else if(state == TRAJ_EXE)
        str="TRAJ_EXE";
    else if(state == GRASP_ANLYS)
        str="GRASP_ANLYS";
    else if(state == ERR)
        str="ERR";
    else if(state == DONE)
        str="DONE";
    else
        str="*unknown state*";
    return str;
}
