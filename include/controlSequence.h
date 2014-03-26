#ifndef CONTROLSEQUENCE_H
#define CONTROLSEQUENCE_H

#include <tabletop_object_detector/TabletopSegmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

#include "tubeManipulation.h"
#include "tubePerception.h"
#include "tubeGrasp.h"
#include "utility.h"

class ControlSequence
{
public:
    ControlSequence(ros::NodeHandlePtr nh);
    ~ControlSequence();
    bool initialize();
    void start();

private:
    ros::NodeHandlePtr _nh;
    TubeManipulation::Arms::Ptr _arms;
    TubePerception::Tube::Ptr _tube;
    TubePerception::CloudProcessing::Ptr _cloud_process;
    TubeGrasp::GraspAnalysis::Ptr _grasp_analysis;
    robotHead _pr2_head;
    ros::ServiceClient _seg_srv_client;
    std::vector<sensor_msgs::PointCloud2> _clusters;
    ros::Publisher _tube_mrkr_pub;
    ros::Publisher _collosion_obj_pub;
    TubeGrasp::GraspPair _grasp_pair;
    geometry_msgs::Pose _pick_pose;
    TubeGrasp::GraspPair _current_grasp;
    arm_navigation_msgs::AttachedCollisionObject _att_obj;
    arm_navigation_msgs::CollisionObject _table;
    std::vector<double> _right_arm_home_jnts;
    std::vector<double> _left_arm_home_jnts;
    ros::ServiceClient _set_pln_scn;

    bool _attached_to_right_arm;
    bool _attached_to_left_arm;

    bool _set_cloud_capture_posture();
    bool _get_segmented_cloud();
    bool _generate_tube_model(unsigned int cluster_idx);
    bool _get_grasps();
    bool _pick_up_tube(const std::string byWhichArm);
    bool _repos_tube_and_regrasp();
    void _get_attached_object();
    void _get_trajectory();
    bool _move_to_staging_point();
    void _get_table_as_object(tabletop_object_detector::TabletopSegmentation &seg_srv);
    bool _move_arm_to_home_position(std::string which_arm);
    void _set_planning_scene();
};


#endif // CONTROLSEQUENCE_H
