#include "collisionObjects.h"

#define CL_LGRNM "clObj"

collisionObjects::collisionObjects(ros::NodeHandlePtr nh)
{
    _nh = nh;
    _set_pln_scn_client = _nh->serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
    setPlanningScene();
}

collisionObjects::~collisionObjects()
{
    ;
}

void collisionObjects::copyAllObjectsTo(collisionObjects::Ptr anotherObj){
    for(unsigned int i=0; i<_att_obj.size(); i++){
        anotherObj->addAttachedCollisionObject(_att_obj[i]);
    }
    for(unsigned int i=0; i<_coll_obj.size(); i++){
        anotherObj->addCollisionObject(_coll_obj[i]);
    }
    if(!_allowed_contact.shape.dimensions.empty()){
        anotherObj->setAllowedContactCube(_allowed_contact.pose_stamped.pose, _allowed_contact.shape.dimensions.at(0), _allowed_contact.link_names);
    }
}

void collisionObjects::addAttachedCollisionObject(const arm_navigation_msgs::AttachedCollisionObject &attObj)
{
    int idx = _find_attached_object(attObj.object.id.c_str());
    if(idx > -1 && idx<_att_obj.size()){
        ROS_DEBUG_NAMED(CL_LGRNM,"Requested to add %s as attached object but it already exists. Updating object", attObj.object.id.c_str());
        _att_obj[idx] = attObj;
        return;
    }
    else if(!attObj.object.id.empty()){
        ROS_INFO_NAMED(CL_LGRNM,"Adding %s attached object", attObj.object.id.c_str());
        _att_obj.push_back(attObj);
    }
}

void collisionObjects::addCollisionObject(const arm_navigation_msgs::CollisionObject &obj)
{
    int idx = _find_collision_object(obj.id.c_str());
    if(idx > -1 && idx < _coll_obj.size()){
        ROS_DEBUG_NAMED(CL_LGRNM,"Requested to add %s as collsion object but it already exists. Updating object", obj.id.c_str());
        _coll_obj[idx] = obj;
        return;
    }
    else if(!obj.id.empty()){
        ROS_INFO_NAMED(CL_LGRNM,"Adding %s object", obj.id.c_str());
        _coll_obj.push_back(obj);
    }
}

void collisionObjects::removeAttachedCollisionObject(std::string id){
    int idx = _find_attached_object(id);
    if(idx>-1 && idx<_att_obj.size()){
        ROS_INFO_NAMED(CL_LGRNM, "Removing %s from the attached collision object list", _att_obj[idx].object.id.c_str());
        _att_obj.erase(_att_obj.begin()+idx);
    }
}

void collisionObjects::removeCollisionObject(std::string id){
    int idx = _find_collision_object(id);
    if(idx>-1 && idx<_coll_obj.size()){
        ROS_INFO_NAMED(CL_LGRNM, "Removing %s from the collision object list", _coll_obj[idx].id.c_str());
        _coll_obj.erase(_coll_obj.begin()+idx);
    }
}

void collisionObjects::setAllowedContactCube(geometry_msgs::Pose pose, double dim, std::vector<std::string> &link_names){
    _allowed_contact.name = "AllowedContact";
    _allowed_contact.link_names.clear();
    _allowed_contact.link_names.push_back("tube");
    _allowed_contact.link_names.push_back("disk");
    _allowed_contact.penetration_depth = -1;
    _allowed_contact.pose_stamped.header.frame_id = "/base_link";
    _allowed_contact.pose_stamped.header.stamp = ros::Time::now();
    _allowed_contact.pose_stamped.pose = pose;
    _allowed_contact.shape.dimensions.resize(3);
    _allowed_contact.shape.dimensions[0] = dim;
    _allowed_contact.shape.dimensions[1] = dim;
    _allowed_contact.shape.dimensions[2] = dim;
    _allowed_contact.shape.type = _allowed_contact.shape.BOX;
}

void collisionObjects::clearAllowedContact(){
    _allowed_contact.shape.dimensions.clear();
}

arm_navigation_msgs::SetPlanningSceneDiff::Response collisionObjects::setPlanningScene(){
    arm_navigation_msgs::SetPlanningSceneDiff::Request req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response res;

    for(unsigned int i=0; i<_att_obj.size(); i++){
        if(!_att_obj[i].object.shapes.empty()){
            req.planning_scene_diff.attached_collision_objects.push_back(_att_obj[i]);
        }
    }

    for(unsigned int i=0; i<_coll_obj.size(); i++){
        if(!_coll_obj[i].shapes.empty()){
            req.planning_scene_diff.collision_objects.push_back(_coll_obj[i]);
        }
    }

    if(!_allowed_contact.shape.dimensions.empty()){
        req.planning_scene_diff.allowed_contacts.push_back(_allowed_contact);
    }

    if(!_set_pln_scn_client.call(req, res)){
        ROS_ERROR_NAMED(CL_LGRNM,"Call to %s failed", SET_PLANNING_SCENE_DIFF_NAME);
        if(!ros::service::exists(SET_PLANNING_SCENE_DIFF_NAME, true)){
            ROS_WARN_NAMED(CL_LGRNM,"%s service doesn't exist", SET_PLANNING_SCENE_DIFF_NAME);
        }
    }
    //printListOfObjects();
    return res;
}

void collisionObjects::clearAllObjects(void){
    _att_obj.clear();
    _coll_obj.clear();
}

void collisionObjects::printListOfObjects(){
    std::cout<<"\n*********************************";
    std::cout<<'\n'<<"Attached Collision Objects:"<<_att_obj.size();
    for(unsigned int i=0; i<_att_obj.size(); i++){
        std::cout<<"\n\t"<<_att_obj[i].object.id<<" with "<<_att_obj[i].object.shapes.size()<<" shapes";
    }
    std::cout<<"\nCollision Objects:"<<_coll_obj.size();
    for(unsigned int i=0; i<_coll_obj.size(); i++){
        std::cout<<"\n\t"<<_coll_obj[i].id<<" with "<<_coll_obj[i].shapes.size()<<" shapes";
    }
    std::cout<<"\n*********************************\n";
}

int collisionObjects::_find_attached_object(std::string id){
    for(unsigned int i=0; i<_att_obj.size(); i++){
        if(_att_obj[i].object.id.compare(id.c_str())==0){
            return i;
        }
    }
    return -1;
}

int collisionObjects::_find_collision_object(std::string id){
    for(unsigned int i=0; i<_coll_obj.size(); i++){
        if(_coll_obj[i].id.compare(id.c_str())==0){
            return i;
        }
    }
    return -1;
}

