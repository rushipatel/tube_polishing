#ifndef COLLISIONOBJECTS_H
#define COLLISIONOBJECTS_H

#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <ros/ros.h>

#define SET_PLANNING_SCENE_DIFF_NAME "/environment_server/set_planning_scene_diff"

class collisionObjects
{
public:
    collisionObjects(ros::NodeHandlePtr nh);
    ~collisionObjects();
    // adds object and if already exists with same id updates that object
    void addCollisionObject(const arm_navigation_msgs::CollisionObject &obj);
    void addAttachedCollisionObject(const arm_navigation_msgs::AttachedCollisionObject &attObj);
    void clearAllObjects(void);
    void removeAttachedCollisionObject(std::string id);
    void removeCollisionObject(std::string id);
    void setAllowedContactCube(geometry_msgs::Pose pose, double dim, std::vector<std::string> &link_names);
    void clearAllowedContact(void);
    // sets planning scene using attached and collision objects
    arm_navigation_msgs::SetPlanningSceneDiff::Response setPlanningScene();
    // resets scene to default
    bool resetPlanningScene();
    void printListOfObjects();
    typedef boost::shared_ptr<collisionObjects> Ptr;
    void copyAllObjectsTo(collisionObjects::Ptr anotherObj);

private:
    std::vector<arm_navigation_msgs::AttachedCollisionObject> _att_obj;
    std::vector<arm_navigation_msgs::CollisionObject> _coll_obj;
    int _find_attached_object(std::string id);
    int _find_collision_object(std::string id);
    arm_navigation_msgs::AllowedContactSpecification _allowed_contact;

    ros::NodeHandlePtr _nh;
    ros::ServiceClient _set_pln_scn_client; /*!< set planning scene diff */
};

#endif // COLLISIONOBJECTS_H
