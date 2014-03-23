#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <ros/ros.h>

#include "tubePerception.h"
#include "tubeManipulation.h"
#include "tubeGrasp.h"

class stateMachine
{
public:
    enum{
        INIT,
        PERCIEVE,
        PICK,
        REGRASP,
        TRAJ_GEN,
        TRAJ_EXE
    };

private:
    bool _att2right, _att2left;
    int _state;
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
};


#endif // STATE_MACHINE_H
