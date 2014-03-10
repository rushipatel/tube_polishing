#include "tubeManipulation.h"

TubeManipulation::CollisionCheck::CollisionCheck(ros::NodeHandlePtr nh)
{
    _nh = nh;
    _mrkr_pub = _nh->advertise<visualization_msgs::MarkerArray>("tube_polishing/state_validity",100);
    _collision_models = new planning_environment::CollisionModels("robot_description");
    ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
    _get_scn_client = _nh->serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);

    _visualize = false;
    _good_color.a = _collision_color.a = _joint_limits_color.a = .8;
    _good_color.g = 1.0;
    _collision_color.r = 1.0;
    _joint_limits_color.b = 1.0;
    _point_markers.a = 1.0;
    _point_markers.r = 1.0;
    _point_markers.g = .8;

    _r_jnts.resize(7);
    _l_jnts.resize(7);
    _r_jnt_nms.resize(7);
    _l_jnt_nms.resize(7);

    /*_jnt_values["r_shoulder_pan_joint"] = 0;
    _jnt_values["r_shoulder_lift_joint"] = 0;
    _jnt_values["r_upper_arm_roll_joint"] = 0;
    _jnt_values["r_elbow_flex_joint"] = 0;
    _jnt_values["r_forearm_roll_joint"] = 0;
    _jnt_values["r_wrist_flex_joint"] = 0;
    _jnt_values["r_wrist_roll_joint"] = 0;*/

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

    _reset_state();
}

void TubeManipulation::CollisionCheck::enableVisualization()
{
    _visualize = true;
}
void TubeManipulation::CollisionCheck::disableVisualization()
{
    _visualize = false;
}

TubeManipulation::CollisionCheck::~CollisionCheck()
{
        _collision_models->revertPlanningScene(_state);
}

void TubeManipulation::CollisionCheck::setAttachedObj(arm_navigation_msgs::AttachedCollisionObject &attachedObj)
{
    _att_obj = attachedObj;
    refreshState();
}

void TubeManipulation::CollisionCheck::clearAttachedObj()
{
    arm_navigation_msgs::AttachedCollisionObject obj;
    _att_obj = obj;
    refreshState();
}

void TubeManipulation::CollisionCheck::refreshState(void)
{
    _collision_models->revertPlanningScene(_state);
    _reset_state();
}

void TubeManipulation::CollisionCheck::_reset_state(void)
{

    if(!_att_obj.object.shapes.empty())
        _scn_req.planning_scene_diff.attached_collision_objects.push_back(_att_obj);
    else
        _scn_req.planning_scene_diff.attached_collision_objects.clear();

    if(!_get_scn_client.call(_scn_req,_scn_res))
    {
        ROS_ERROR("Can't get planning scene");
        return;
    }
    _scn = _scn_res.planning_scene;
    _state = _collision_models->setPlanningScene(_scn);
    _r_jnt_nms = _collision_models->getKinematicModel()->getModelGroup("right_arm")->getJointModelNames();
    _l_jnt_nms = _collision_models->getKinematicModel()->getModelGroup("left_arm")->getJointModelNames();
    _r_lnk_nms = _collision_models->getKinematicModel()->getModelGroup("right_arm")->getUpdatedLinkModelNames();
    _l_lnk_nms = _collision_models->getKinematicModel()->getModelGroup("left_arm")->getUpdatedLinkModelNames();

    _state->getJointStateGroup("right_arm")->getKinematicStateValues(_jnt_values);
    _actual_r_jnts.resize(_r_jnt_nms.size());
    for(size_t i=0; i<_r_jnt_nms.size(); i++)
        _actual_r_jnts[i] = _jnt_values[_r_jnt_nms[i].c_str()];

    _actual_l_jnts.resize(_l_jnt_nms.size());
    _state->getJointStateGroup("left_arm")->getKinematicStateValues(_jnt_values);
    for(size_t i=0; i<_l_jnt_nms.size(); i++)
        _actual_l_jnts[i] = _jnt_values[_l_jnt_nms[i].c_str()];
}

void TubeManipulation::CollisionCheck::printState(void)
{
    refreshState();
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

    std::cout<<"Joint Values:"<<std::endl;
    std::cout<<"    \tRight"<<"\t\t\t"<<"Left"<<std::endl;
    for(int i=0; i<_actual_r_jnts.size(); i++)
    {
        std::cout<<i<<" > \t"<<_actual_r_jnts[i]<<"\t\t"<<_actual_l_jnts[i]<<std::endl;
    }
    std::cout<<std::endl;
}

bool TubeManipulation::CollisionCheck::isStateValid(std::vector<double> &right_joints, std::vector<double> &left_joints)
{
    ROS_ASSERT_MSG(right_joints.size()==7,"TubeManipulation - Number of right joint values is not 7");
    ROS_ASSERT_MSG(left_joints.size()==7,"TubeManipulation - Number of left joint values is not 7");

    _r_jnts = right_joints;
    _l_jnts = left_joints;

    _jnt_values.clear();

    for(size_t i=0; i<_r_jnts.size(); i++)
    {
        _jnt_values[_r_jnt_nms[i]] = _r_jnts[i];
        _jnt_values[_l_jnt_nms[i]] = _l_jnts[i];
    }
    return _is_state_valid();
}

bool TubeManipulation::CollisionCheck::_is_state_valid()
{

    bool is_valid = true;

    std_msgs::ColorRGBA color;
    color = _good_color;
    double mrk_life_time = 10;

    _state->setKinematicState(_jnt_values);
    if(!_state->areJointsWithinBounds(_r_jnt_nms))
    {
        ROS_WARN("TubeManipulation - Right joints are out of bound");
        color = _joint_limits_color;
        is_valid = false;
    }
    if(!_state->areJointsWithinBounds(_l_jnt_nms))
    {
        ROS_WARN("TubeManipulation - Left joints are out of bound");
        color = _joint_limits_color;
        is_valid = false;
    } else if(_collision_models->isKinematicStateInCollision(*_state))
    {
        std::stringstream ss;
        color = _collision_color;
        is_valid = false;
        if(_collision_models->isKinematicStateInEnvironmentCollision(*_state))
            ss<<"environment ";
        if(_collision_models->isKinematicStateInSelfCollision(*_state))
        {
            if(ss.gcount()>2)
                ss<<"and self";
            else
                ss<<"self";
        }
        ROS_WARN_STREAM("TubeManipulation - kinematic state is in collision with "<<ss.str());
        _collision_models->getAllCollisionPointMarkers(*_state, _mrkr_arr, _point_markers, ros::Duration(mrk_life_time));
    }

    if(_visualize)
    {
        _collision_models->getRobotMarkersGivenState(*_state, _mrkr_arr, color,"right_arm",ros::Duration(mrk_life_time),&_r_lnk_nms);
        _collision_models->getRobotMarkersGivenState(*_state, _mrkr_arr, color,"left_arm",ros::Duration(mrk_life_time),&_l_lnk_nms);
        _collision_models->getAttachedCollisionObjectMarkers(*_state, _mrkr_arr, "right_arm",color,ros::Duration(mrk_life_time), false, &_r_lnk_nms);
        _collision_models->getAttachedCollisionObjectMarkers(*_state, _mrkr_arr, "left_arm",color,ros::Duration(mrk_life_time), false, &_l_lnk_nms);
        _mrkr_pub.publish(_mrkr_arr);
    }
    return is_valid;
}
