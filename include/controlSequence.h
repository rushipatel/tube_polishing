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
    TubeGrasp::GraspPair _grasp_pair;
    geometry_msgs::Pose _pick_pose;
    TubeGrasp::GraspPair _current_grasp;
    arm_navigation_msgs::AttachedCollisionObject _att_obj;

    bool _attached_to_right_arm;
    bool _attached_to_left_arm;

    bool _set_cloud_capture_posture();
    bool _get_segmented_cloud();
    bool _generate_tube_model(unsigned int cluster_idx);
    bool _get_grasps();
    bool _pick_up_tube(const std::string byWichArm);
    bool _repos_tube_and_regrasp();
    void _get_attached_object();
    void _get_trajectory();
    bool _move_to_staging_point();
};


#endif // CONTROLSEQUENCE_H
