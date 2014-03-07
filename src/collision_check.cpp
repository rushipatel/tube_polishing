#include "tubeManipulation.h"


TubeManipulation::CollisionCheck::CollisionCheck(ros::NodeHandlePtr nh)
{
    _nh = nh;
    _mrkr_pub = _nh->advertise<visualization_msgs::MarkerArray>("tube_polishing/state_validity",100);
    _collision_models = new planning_environment::CollisionModels("robot_description");
    ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
    _get_scn_client = _nh->serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);

    _r_jnts.resize(7);
    _l_jnts.resize(7);
    _r_jnt_nms.resize(7);
    _l_jnt_nms.resize(7);
    _r_lnk_nms.resize(7);
    _l_lnk_nms.resize(7);

    _jnt_values["r_shoulder_pan_joint"] = 0;
    _jnt_values["r_shoulder_lift_joint"] = 0;
    _jnt_values["r_upper_arm_roll_joint"] = 0;
    _jnt_values["r_elbow_flex_joint"] = 0;
    _jnt_values["r_forearm_roll_joint"] = 0;
    _jnt_values["r_wrist_flex_joint"] = 0;
    _jnt_values["r_wrist_roll_joint"] = 0;

    /*_r_jnt_nms.push_back("r_shoulder_pan_joint");
    _r_jnt_nms.push_back("r_shoulder_lift_joint");
    _r_jnt_nms.push_back("r_upper_arm_roll_joint");
    _r_jnt_nms.push_back("r_elbow_flex_joint");
    _r_jnt_nms.push_back("r_forearm_roll_joint");
    _r_jnt_nms.push_back("r_wrist_flex_joint");
    _r_jnt_nms.push_back("r_wrist_roll_joint");

    _l_jnt_nms.push_back("l_shoulder_pan_joint");
    _l_jnt_nms.push_back("l_shoulder_lift_joint");
    _l_jnt_nms.push_back("l_upper_arm_roll_joint");
    _l_jnt_nms.push_back("l_elbow_flex_joint");
    _l_jnt_nms.push_back("l_forearm_roll_joint");
    _l_jnt_nms.push_back("l_wrist_flex_joint");
    _l_jnt_nms.push_back("l_wrist_roll_joint");

    _r_lnk_nms.push_back("r_shoulder_pan_link");
    _r_lnk_nms.push_back("r_shoulder_lift_link");
    _r_lnk_nms.push_back("r_upper_arm_roll_link");
    _r_lnk_nms.push_back("r_elbow_flex_joint");
    _r_lnk_nms.push_back("r_forearm_roll_joint");
    _r_lnk_nms.push_back("r_wrist_flex_joint");
    _r_lnk_nms.push_back("r_wrist_roll_joint");

    _l_lnk_nms.push_back("l_shoulder_pan_joint");
    _l_lnk_nms.push_back("l_shoulder_lift_joint");
    _l_lnk_nms.push_back("l_upper_arm_roll_joint");
    _l_lnk_nms.push_back("l_elbow_flex_joint");
    _l_lnk_nms.push_back("l_forearm_roll_joint");
    _l_lnk_nms.push_back("l_wrist_flex_joint");
    _l_lnk_nms.push_back("l_wrist_roll_joint");*/

    resetState();
}

TubeManipulation::CollisionCheck::~CollisionCheck()
{
        _collision_models->revertPlanningScene(_state);
}

void TubeManipulation::CollisionCheck::resetState(void)
{
    //_collision_models->revertPlanningScene(_state);
    if(!_att_obj.object.shapes.empty())
        _scn_req.planning_scene_diff.attached_collision_objects.push_back(_att_obj);

    if(!_get_scn_client.call(_scn_req,_scn_res))
    {
        ROS_ERROR("Can't get planning scene");
        return;
    }
    _scn = _scn_res.planning_scene;
    _state = _collision_models->setPlanningScene(_scn);
    //_actual_right_joints = _state->getJointState("right_arm");
}

void TubeManipulation::CollisionCheck::printState(void)
{
    _r_jnt_nms = _collision_models->getKinematicModel()->getModelGroup("right_arm")->getJointModelNames();
    _state->getJointStateGroup("right_arm")->getKinematicStateValues(_jnt_values);
    _l_jnt_nms = _collision_models->getKinematicModel()->getModelGroup("left_arm")->getJointModelNames();
    _r_lnk_nms = _collision_models->getKinematicModel()->getModelGroup("right_arm")->getUpdatedLinkModelNames();
    _l_lnk_nms = _collision_models->getKinematicModel()->getModelGroup("left_arm")->getUpdatedLinkModelNames();

    std::cout<<std::endl;
    std::cout<<"****** State Output *******"<<std::endl;

    std::cout<<"Right Joint Names:"<<std::endl;
    for(int i=0; i<_r_jnt_nms.size(); i++)
        std::cout<<i<<"  "<<_r_jnt_nms[i]<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Left Joint Names:"<<std::endl;
    for(int i=0; i<_l_jnt_nms.size(); i++)
        std::cout<<i<<"  "<<_l_jnt_nms[i]<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Right Link Names:"<<std::endl;
    for(int i=0; i<_r_lnk_nms.size(); i++)
        std::cout<<i<<"  "<<_r_lnk_nms[i]<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Left Link Names:"<<std::endl;
    for(int i=0; i<_l_lnk_nms.size(); i++)
        std::cout<<i<<"  "<<_l_lnk_nms[i]<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Right Joint Values:"<<std::endl;
    for(int i=0; i<_r_jnt_nms.size(); i++)
        std::cout<<i<<" > "<<_r_jnt_nms[i].c_str()<<" : "<<_jnt_values[_r_jnt_nms[i].c_str()]<<std::endl;
    std::cout<<std::endl;

}

//bool TubeManipulation::CollisionCheck::_is_state_valid(std::vector<double> &right_joints,
//                                       std::vector<double> &left_joints,
//                                       arm_navigation_msgs::GetPlanningScene::Request req)
//{

//    bool is_valid = true;

//    arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;
//    if(!_get_pln_scn_client.call(req, planning_scene_res))
//        ROS_ERROR("Can't get planning scene");

//    planning_environment::CollisionModels collision_models("robot_description");
//    planning_models::KinematicState* state =
//            collision_models.setPlanningScene(planning_scene_res.planning_scene);

//    std::vector<std::string> right_joint_names =
//            collision_models.getKinematicModel()->getModelGroup("right_arm")->getJointModelNames();
//    std::vector<std::string> right_link_names =
//            collision_models.getKinematicModel()->getModelGroup("right_arm")->getUpdatedLinkModelNames();
//    std::vector<std::string> left_joint_names =
//            collision_models.getKinematicModel()->getModelGroup("left_arm")->getJointModelNames();
//    std::vector<std::string> left_link_names =
//            collision_models.getKinematicModel()->getModelGroup("left_arm")->getUpdatedLinkModelNames();

//    std::map<std::string, double> joint_values;
//    ROS_ASSERT_MSG(right_joints.size()==7,"Number of right joint values is not 7");
//    ROS_ASSERT_MSG(left_joints.size()==7,"Number of left joint values is not 7");

//    joint_values["r_shoulder_pan_joint"] = right_joints[0];
//    joint_values["r_shoulder_lift_joint"] = right_joints[1];
//    joint_values["r_upper_arm_roll_joint"] = right_joints[2];
//    joint_values["r_elbow_flex_joint"] = right_joints[3];
//    joint_values["r_forearm_roll_joint"] = right_joints[4];
//    joint_values["r_wrist_flex_joint"] = right_joints[5];
//    joint_values["r_wrist_roll_joint"] = right_joints[6];

//    joint_values["l_shoulder_pan_joint"] = left_joints[0];
//    joint_values["l_shoulder_lift_joint"] = left_joints[1];
//    joint_values["l_upper_arm_roll_joint"] = left_joints[2];
//    joint_values["l_elbow_flex_joint"] = left_joints[3];
//    joint_values["l_forearm_roll_joint"] = left_joints[4];
//    joint_values["l_wrist_flex_joint"] = left_joints[5];
//    joint_values["l_wrist_roll_joint"] = left_joints[6];


//    /*collision_space::EnvironmentModel::AllowedCollisionMatrix new_coll_matrix =
//            collision_space::EnvironmentModel::AllowedCollisionMatrix(
//                    collision_models.getDefaultAllowedCollisionMatrix());
//    new_coll_matrix.changeEntry("r_gripper_r_finger_tip_link", "collision_map", true);
//    new_coll_matrix.changeEntry("collision_map", "r_gripper_r_finger_tip_link", true);
//    new_coll_matrix.changeEntry("r_gripper_l_finger_tip_link", "collision_map", true);
//    new_coll_matrix.changeEntry("collision_map", "r_gripper_l_finger_tip_link", true);
//    new_coll_matrix.changeEntry("l_gripper_r_finger_tip_link", "collision_map", true);
//    new_coll_matrix.changeEntry("collision_map", "l_gripper_r_finger_tip_link", true);
//    new_coll_matrix.changeEntry("l_gripper_l_finger_tip_link", "collision_map", true);
//    new_coll_matrix.changeEntry("collision_map", "l_gripper_l_finger_tip_link", true);
//    collision_models.setAlteredAllowedCollisionMatrix(new_coll_matrix);*/


//    /*planning_models::KinematicState::JointState *js_ptr;
//    std::vector<double> j_vals;
//    ROS_INFO("values after setKineState");
//    for(int i=0; i<joint_names.size(); i++)
//    {
//        js_ptr = state->getJointState(joint_names[i].c_str());
//        j_vals = js_ptr->getJointStateValues();
//        if(j_vals.size()>1)
//            ROS_WARN("j_vals size is grater than 1");
//        ROS_INFO_STREAM(" "<<joint_names[i].c_str()<<" = "<<j_vals[0]);
//    }*/

//    std_msgs::ColorRGBA good_color, collision_color, joint_limits_color;
//    good_color.a = collision_color.a = joint_limits_color.a = .8;
//    good_color.g = 1.0;
//    collision_color.r = 1.0;
//    joint_limits_color.b = 1.0;

//    std_msgs::ColorRGBA point_markers;
//    point_markers.a = 1.0;
//    point_markers.r = 1.0;
//    point_markers.g = .8;

//    std_msgs::ColorRGBA color;
//    color = good_color;
//    visualization_msgs::MarkerArray arr;

//    state->setKinematicState(joint_values);
//    if(!state->areJointsWithinBounds(right_joint_names))
//    {
//        ROS_WARN("TubeManipulation - Right joints are out of bound");
//        color = joint_limits_color;
//        is_valid = false;
//    }
//    if(!state->areJointsWithinBounds(left_joint_names))
//    {
//        ROS_WARN("TubeManipulation - Left joints are out of bound");
//        color = joint_limits_color;
//        is_valid = false;
//    }
//    if(!collision_models.isKinematicStateInCollision(*state))
//    {
//        ROS_WARN("TubeManipulation - kinematic state is in collision");
//        color = collision_color;
//        is_valid = false;
//    }
//    std::vector<arm_navigation_msgs::ContactInformation> contacts;
//    collision_models.getAllCollisionsForState(*state, contacts, 1);
//    double mrk_life_time = 10;
//    collision_models.getAllCollisionPointMarkers(*state, arr, point_markers, ros::Duration(mrk_life_time));
//    collision_models.getAttachedCollisionObjectMarkers(*state, arr, "right_arm",color,ros::Duration(mrk_life_time));
//    collision_models.getAttachedCollisionObjectMarkers(*state, arr, "left_arm",color,ros::Duration(mrk_life_time));
//    collision_models.getRobotMarkersGivenState(*state,arr,color,"right_arm",ros::Duration(mrk_life_time),&right_link_names);
//    collision_models.getRobotMarkersGivenState(*state,arr,color,"left_arm",ros::Duration(mrk_life_time),&left_link_names);

//    _scene_pub.publish(arr);
//    //collision_models.revertPlanningScene(state);
//    delete state;
//    return is_valid;
//}

//bool TubeManipulation::CollisionCheck::isStateValid(std::vector<double> &right_joints, std::vector<double> &left_joints)
//{
//    arm_navigation_msgs::GetPlanningScene::Request req;

//    if(right_joints.empty())
//        _get_right_joints(right_joints);
//    if(left_joints.empty())
//        _get_left_joints(left_joints);
//    if(!_is_state_valid(right_joints, left_joints, req))
//        return false;
//    ROS_INFO("State is valid");
//    return true;
//}

//bool TubeManipulation::CollisionCheck::isStateValid(arm_navigation_msgs::AttachedCollisionObject attachedObj)
//{
//    arm_navigation_msgs::GetPlanningScene::Request req;
//    req.planning_scene_diff.attached_collision_objects.push_back(attachedObj);

//    std::vector<double> right_joints, left_joints;
//    _get_right_joints(right_joints);
//    _get_left_joints(left_joints);
//    if(!_is_state_valid(right_joints, left_joints, req))
//        return false;
//    ROS_INFO("State is valid");
//    return true;
//}

//bool TubeManipulation::CollisionCheck::isStateValid(arm_navigation_msgs::AttachedCollisionObject attachedObj, std::vector<double> &right_joints, std::vector<double> &left_joints)
//{
//    arm_navigation_msgs::GetPlanningScene::Request req;
//    req.planning_scene_diff.attached_collision_objects.push_back(attachedObj);
//    if(!_is_state_valid(right_joints, left_joints, req))
//        return false;
//    ROS_INFO("State is valid");
//    return true;
//}

//bool TubeManipulation::CollisionCheck::_get_regrasp_pose_right(geometry_msgs::Pose crnt_grasp,
//                                               geometry_msgs::Pose wrist_pose,
//                                               geometry_msgs::Pose right_grasp,
//                                               geometry_msgs::Pose left_grasp,
//                                               geometry_msgs::Pose &obj_pose_out)
//{
//    tf::Transform obj_orig,obj,rand_tf,
//            wrist=pose2tf(wrist_pose),
//            grasp=pose2tf(crnt_grasp),
//            rg = pose2tf(right_grasp),
//            lg = pose2tf(left_grasp);

//    obj_orig = wrist * grasp.inverse();

//    tf::Quaternion q;
//    tf::Vector3 pos;
//    double y,p,r;
//    int cnt = 1000;
//    obj = obj_orig;
//    std::vector<double> right_joints(7), right_seeds(7),
//                        left_seeds(7), left_joints(7);

//    arm_navigation_msgs::GetPlanningScene::Request req;

//    _get_right_joints(right_seeds);
//    _get_left_joints(left_seeds);
//    tf::Transform right_wrist, right_wrist_crnt, left_wrist;
//    geometry_msgs::Pose right_wrist_crnt_pose, right_wrist_pose, left_wrist_pose;

//    do
//    {
//        right_wrist_crnt = obj * grasp;
//        right_wrist = obj * rg;
//        left_wrist = obj * lg;
//        right_wrist_pose = tf2pose(right_wrist);
//        right_wrist_crnt_pose = tf2pose(right_wrist_crnt);
//        left_wrist_pose = tf2pose(left_wrist);
//        if(_get_simple_right_arm_ik(right_wrist_pose, right_joints, right_seeds)
//           &&_get_simple_right_arm_ik(right_wrist_crnt_pose, right_joints, right_seeds)
//           &&_get_simple_left_arm_ik(left_wrist_pose, left_joints, left_seeds) )
//        {
//            if(_is_state_valid(right_joints, left_joints, req))
//            {
//                ROS_INFO("Valid pose found for regrasp");
//                cnt = 0;
//                obj_orig = obj;
//            }
//        }

//        y = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
//        p = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
//        r = ((double)rand()/(double)RAND_MAX) * 2 * M_PI;
//        q.setRPY(r,p,y);
//        pos.setZero();
//        rand_tf.setRotation(q);
//        rand_tf.setOrigin(pos);
//        obj = obj_orig * rand_tf;
//        std::cout<<cnt<<" ";
//        cnt--;
//        }while(cnt>0);

//    obj_pose_out = tf2pose(obj_orig);
//}
