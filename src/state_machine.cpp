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
    _PICK_WRIST_OFFSET = 0.2;
    _TABLE_HEIGHT = 0.5; //in meters
    _z_error = 0.0;

    _table_obj_id = "Table";
    _att_obj_id = "Tube";
    _tube_obj_id = "staticTube";
    _wheel_id = "Wheel";

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
    _work_pose_pub = _nh->advertise<geometry_msgs::PoseStamped>("/tube_polishing/work_pose",2);

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
    _work_pose1.position.z = 0.9;
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
            ros::Duration(3).sleep(); //give time to head to settle
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
                _att_obj->object.shapes.clear();
                _table.shapes.clear();
                _table.poses.clear();
                _tube->reset();
                _wheel.shapes.clear();
                _wheel.poses.clear();
                _update_scene();
                _cluster_idx++;
                if(_cluster_idx>=_clusters.size()){
                    _state = ERR;
                }
                break;
            }
            _get_disk_and_workpose();
            _state = PICK;
            //_state = DONE;
            ROS_INFO_NAMED(LGRNM,"*Generated tube and published table");
            break;
        }
        case GRASP_ANLYS:
        {
            ROS_INFO_NAMED(LGRNM,"*Analyzing Grasps...");
            _collision_objects->clearAllowedContact();
            std::vector<std::string> link_names;
            link_names.push_back(_tube_obj_id);
            link_names.push_back(_wheel_id);
            _collision_objects->setAllowedContactCube(_work_pose1, 0.05, link_names);
            _update_scene();
            //_get_disk_and_work_pose();
            if(!_get_computed_grasp_pair()){
                _work_traj_idx++;
                _state = GRASP_ANLYS;
                if(_work_traj_idx>=_tube->workPointsCluster.size()){
                    ROS_WARN_NAMED(LGRNM,"No more trajectory left");
                    _state = DONE;
                }
                break;
            }
            visualization_msgs::MarkerArray ma;
            _tube->getWorkPointsMarker(ma);
            _work_point_pub.publish(ma);
            _publish_grasps();
            _state = REGRASP;
            break;
        }
        case PICK:
        {
            ROS_INFO_NAMED(LGRNM,"*Picking tube...");
            if(!_get_pick_grasp()){
                ROS_ERROR_NAMED(LGRNM,"No solution of lift grasp found for any arm");
                _state = ERR;
                break;
            }
            if(_r_soln_avail){
                ROS_INFO_NAMED(LGRNM, "Lifting object with right arm...");
                if(_lift_obj_with_right_arm()){
                    _l_soln_avail = false; //done with these variables as of now. reset them
                    _r_soln_avail = false;
                    _state = RECAPTURE;
                    break;
                }
                else{
                    _r_soln_avail = false;
                    break;
                }
            }
            if(_l_soln_avail){
                _state = ERR;
                break;
            }
            _state = ERR;
            break;
        }
        case RECAPTURE:
        {
            ROS_INFO_NAMED(LGRNM, "*Recapturing point cloud...");
            tf::Transform tube_tf = _tube->getTransform();
            std::vector<double> ik_soln2(7);
            if(_att2right & !_att2left){
                if(_arms->moveRightArmToCloudCapturePose(tf::Vector3(0,0,1.5),tube_tf.getOrigin(),0.9, ik_soln2)){
                    if(_arms->moveRightArmWithMPlanning(ik_soln2)){
                        ros::Duration(3).sleep();
                        tf::Transform tube_tf,
                                grsp = _current_right_grasp.getWristTransform(),
                                fk = pose2tf(_arms->getRightArmFK(ik_soln2));
                        tube_tf = fk*grsp.inverse();
                        geometry_msgs::Pose tube_pose = tf2pose(tube_tf);
                        _tube->setPose(tube_pose);
                        _tube->setPoseAsActualPose();
//                        _publish_tube();
                        _update_scene();
                        //sensor_msgs::PointCloud2ConstPtr cloud_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>
                        //                ("/wide_stereo/points2");
                        geometry_msgs::Pose fk_pose = tf2pose(fk);
                        _head.lookAt(fk_pose.position.x, fk_pose.position.y, fk_pose.position.z-0.1);
                        ros::Duration(3).sleep();
                        sensor_msgs::PointCloud2ConstPtr cloud_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>
                                ("/head_mount_kinect/depth_registered/points",ros::Duration(5));
                        sensor_msgs::PointCloud2::Ptr cloud_filtered(new sensor_msgs::PointCloud2);
                        _cloud_process->extractTubePoints(_tube, cloud_ptr, cloud_filtered);
                        _cloud_process->resetPoseOfTube(*cloud_filtered,_tube);
                        tf::Transform tube = _tube->getTransform(),grasp;
                        grasp = tube.inverseTimes(fk);
                        _current_right_grasp.setPose(grasp);
                        _current_right_grasp.setWristOffset(0.00001);
//                        std::cout<<"\nPublishing tube with new pose...";
//                        std::string s;
//                        std::cin>>s;
                        _update_scene();
//                        std::cin>>s;
                        //_publish_tube();
                    }
                    else{
                        ROS_WARN_NAMED(LGRNM,"Couldn't move to point cloud recapture pose!");
                    }
                }
                else{
                    ROS_WARN_NAMED(LGRNM, "Couldn't get cloud capture pose!");
                    ROS_INFO_NAMED(LGRNM, "Do you want to try again?\n(y/n)? :");
                    std::string s;
                    std::cin>> s;
                    if(!(s.compare("y")==0)){
                        _state = DONE;
                        break;
                    }
                    else{
                        _state = RECAPTURE;
                        break;
                    }
                }
                _state = GRASP_ANLYS;
                break;
            }
            _state = GRASP_ANLYS;
            break;
        }
        case REGRASP:
        {
            ROS_INFO_NAMED(LGRNM,"*Regrasping...");
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
            ROS_INFO_NAMED(LGRNM,"*Generating trajectory...");
            _update_scene();
            if(_att2left){
                std::vector<double> q_right_first(7), q_left_first(7);
                if(!(_computed_grasp_pair.qLeft.size()>7 && _computed_grasp_pair.qRight.size()>7)){
                    ROS_ERROR_NAMED(LGRNM,"trajectories stored in computed grasp pair is less than 7");
                }
                for(unsigned int i=0; i<7; i++){
                    q_left_first[i] = _computed_grasp_pair.qLeft[i];
                    q_right_first[i] = _computed_grasp_pair.qRight[i];
                }
                if(!_arms->moveLeftArmWithMPlanning(q_left_first)){ //first 7 values
                    _state = ERR;
                    break;
                }
                if(!_arms->moveRightArmWithMPlanning(q_right_first)){
                    _state = ERR;
                    break;
                }
                _gripper->setRightGripperPosition(_tube->cylinders[0].radius*/*1.5*/1, -1);
                _att2right = true;
                ROS_INFO_NAMED(LGRNM,"Executing synced trajectory...");
                if(!_arms->executeJointTrajectoryWithSync(_computed_grasp_pair.qRight, _computed_grasp_pair.qLeft)){
                    _state = ERR;
                    break;
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
//            if(pose.position.z<(_TABLE_HEIGHT+shape.dimensions.at(0))){
//                //ROS_WARN_NAMED(LGRNM,
//                //"z value of static_tube pose origin is less than height of table plus its radius. Adjusting z value");
//                //_tube_collision_obj.poses[i].position.z = _TABLE_HEIGHT+shape.dimensions.at(0);
//            }
        }
        _collision_objects->addCollisionObject(_tube_collision_obj);
    }
    else{
        _collision_objects->removeCollisionObject(_tube_collision_obj.id.c_str());
    }
    if(!_wheel.shapes.empty()){
        _collision_objects->addCollisionObject(_wheel);
    }
    else{
        _collision_objects->removeCollisionObject(_wheel_id);
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

void stateMachine::_get_wheel_collision_object(){
    _wheel.header.frame_id = "/base_link";
    _wheel.header.stamp = ros::Time::now();
    _wheel.id = _wheel_id;
    _wheel.operation.operation = _wheel.operation.ADD;

    geometry_msgs::Pose pose = _wheel_cyl.getPose();
    arm_navigation_msgs::Shape shape;
    shape.type = shape.CYLINDER;
    shape.dimensions.resize(2);;
    shape.dimensions[0] = _wheel_cyl.radius;
    shape.dimensions[1] = _wheel_cyl.getAxisLength();
    _wheel.shapes.clear();
    _wheel.poses.clear();
    _wheel.poses.push_back(pose);
    _wheel.shapes.push_back(shape);
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
    visualization_msgs::Marker marker;
    marker.type = marker.SPHERE;
    marker.action = marker.ADD;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.color.a = 1;
    marker.color.b = marker.color.g = marker.color.r = 1;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.01;
    //marker.pose.orientation.w = 1.0;
    marker.id++;
    marker.pose = _computed_grasp_pair.rightGrasp.getWristGlobalPose(_tube->getPose());
    marker_array.markers.push_back(marker);
    marker.id++;
    marker.pose = _computed_grasp_pair.leftGrasp.getWristGlobalPose(_tube->getPose());
    marker_array.markers.push_back(marker);
    _grasp_mrkr_pub.publish(marker_array);
}

void stateMachine::_publish_pick_pose(void){
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "/base_link";
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose = _pick_grasp.getWristGlobalPose(_tube->getPose());
    _pick_grasp_pub.publish(pose_stamped);
}

void stateMachine::_publish_work_pose(void){
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "/base_link";
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose = _work_pose1;
    _work_pose_pub.publish(pose_stamped);
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
    /*tf::Vector3 vec;
    tf::Transform tube_tf = _tube->getTransform(), wrist_tf;
    wrist_tf = _computed_grasp_pair.rightGrasp.getWristGlobalPose(tube_tf);
    vec = wrist_tf.getOrigin();
    points.push_back(vec);
    wrist_tf = _computed_grasp_pair.leftGrasp.getWristGlobalPose(tube_tf);
    vec = wrist_tf.getOrigin();
    points.push_back(vec); //points to avoid*/
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
    if(!_gripper->setRightGripperPosition(_tube->cylinders[_pick_grasp.cylinderIdx].radius*/*1.5*/1, -1)){
        return false;
    }
    _att2right = true;
    geometry_msgs::Pose fk_pose = _arms->getRightArmFK();
    tf::Transform tube_tf = _tube->getTransform(), grasp_pose;
    grasp_pose = tube_tf.inverseTimes(pose2tf(fk_pose));
    _current_right_grasp.setPose(grasp_pose);
    _current_right_grasp.setWristOffset(0.00001);
    if(!_arms->simpleMoveRightArm(approach_pose)){
        return false;
    }
    tf::Transform tube, wrist=pose2tf(approach_pose), grasp = _current_right_grasp.getWristTransform();
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
        /*_current_right_grasp.setWristOffset(_PICK_WRIST_OFFSET+(_tube->cylinders[_current_right_grasp.cylinderIdx].radius*4));
        geometry_msgs::Pose wrist_pose = _current_right_grasp.getWristGlobalPose(_tube->getPose());
        if(!_arms->moveRightArmWithMPlanning(wrist_pose)){
            return false;
            _att2left = true;
            _att2right = false;
        }*/
        _att2left = true;
        _att2right = false;
        _current_left_grasp =  _computed_grasp_pair.leftGrasp;
        _update_scene();
        _move_arm_to_home_position("right_arm");
    }
    return true;
}

void stateMachine::_get_disk_and_workpose(){
    double MIN_R = 0.06;
    double MAX_R = 0.07;
    if(_clusters.empty()){
        ROS_ERROR_NAMED(LGRNM,"No cluster to find wheel");
    }
    TubePerception::Cylinder wheel;
    geometry_msgs::Pose work_pose;
    for(unsigned int i=0; i<_clusters.size(); i++){
        if(_cloud_process->findDisk(_clusters[i], MIN_R, MAX_R, wheel, work_pose)){
            ROS_INFO_NAMED(LGRNM,"Wheel found at: x=%f, y=%f, z=%f", work_pose.position.x,
                           work_pose.position.y, work_pose.position.z);
            _work_pose1 = work_pose;
            _wheel_cyl = wheel;
            _publish_work_pose();
            _get_wheel_collision_object();
            _update_scene();
            break;
        }
    }
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
