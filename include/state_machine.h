#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <ros/ros.h>
#include <tabletop_object_detector/TabletopSegmentation.h>
#include <sensor_msgs/PointCloud2.h>
//#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include "tubePerception.h"
#include "tubeManipulation.h"
#include "tubeGrasp.h"
#include "collisionObjects.h"

class stateMachine
{
public:
    stateMachine(ros::NodeHandlePtr nh);
    enum{
        INIT,
        PERCIEVE,
        PICK,
        GRASP_ANLYS,
        REGRASP,
        TRAJ_GEN,
        TRAJ_EXE,
        DONE,
        ERR
    };
    void start();

private:
    bool _att2right, _att2left;
    bool _r_soln_avail, _l_soln_avail;
    int _state;
    int _MAX_REGRASP_TRY;
    int _regrasp_try;
    int _cluster_idx;
    int _work_traj_idx;
    double _WRIST_OFFSET;
    double _PICK_WRIST_OFFSET;
    double _TABLE_HEIGHT;
    double _z_error;
    std::string _table_obj_id;
    std::string _att_obj_id;
    std::string _tube_obj_id;
    ros::NodeHandlePtr _nh;

    TubeManipulation::Arms::Ptr _arms;
    TubePerception::Tube::Ptr _tube;
    TubePerception::CloudProcessing::Ptr _cloud_process;
    TubeGrasp::GraspAnalysis::Ptr _grasp_analysis;
    robotHead _head;
    Gripper::Ptr _gripper;
    TubeGrasp::GraspPair _computed_grasp_pair;
    TubeGrasp::Grasp _current_right_grasp;
    TubeGrasp::Grasp _current_left_grasp;
    TubeManipulation::CollisionCheck::Ptr _collision_check;
    collisionObjects::Ptr _collision_objects;
    TubeGrasp::Grasp _pick_grasp;

    ros::ServiceClient _seg_srv_client;
    //ros::ServiceClient _set_pln_scn;

    ros::Publisher _tube_mrkr_pub;
    ros::Publisher _collision_obj_pub;
    ros::Publisher _grasp_mrkr_pub;
    ros::Publisher _work_point_pub;
    ros::Publisher _pick_grasp_pub;

    std::vector<sensor_msgs::PointCloud2> _clusters;

    TubeGrasp::GraspPair _crnt_grasp_pair;
    arm_navigation_msgs::AttachedCollisionObject::Ptr _att_obj;
    arm_navigation_msgs::CollisionObject _table;
    arm_navigation_msgs::CollisionObject _tube_collision_obj;
    std::vector<double> _right_arm_home_jnts;
    std::vector<double> _left_arm_home_jnts;
    geometry_msgs::Pose _work_pose1, _work_pose2;

    void _print_state();
    std::string _get_state_str(int state);
    bool _move_arm_to_home_position(std::string which_arm);
    void _update_scene(void);
    bool _get_clusters(void);
    bool _gen_tube_model(void);
    void _extract_table_from_msg(tabletop_object_detector::TabletopSegmentation &seg_srv);
    void _publish_tube(void);
    void _publish_grasps(void);
    void _publish_pick_pose(void);
    bool _get_computed_grasp_pair(void);
    bool _get_pick_grasp(void);
    bool _lift_obj_with_right_arm(void);
    bool _lift_obj_with_left_arm(void);
//    void _add_tube_to_collision_space(void);
//    void _remove_tube_from_collision_space(void);
//    void _add_table_to_collision_space(void);
//    void _remove_table_from_collision_space(void);
    void _get_attached_obj();
    bool _regrasp();
};


#endif // STATE_MACHINE_H
