#ifndef TUBEGRASP_H
#define TUBEGRASP_H

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

#include "tubePerception.h"
#include "manipAnalysis.h"
#include "utility.h"
#include "tubeManipulation.h"
#include "gripper.h"
#include "robotHead.h"
#include "grasp.h"

namespace TubeGrasp
{
    class GraspAnalysis
    {
    public:
        ros::NodeHandlePtr nodeHandle;
        GraspAnalysis(TubePerception::Tube::Ptr tube, ros::NodeHandlePtr nh);
        void setWorkPose(geometry_msgs::Pose &p);
        int getWorkPose(geometry_msgs::Pose &p);
        void setWorkTrajIdx(int trajIdx);
        //void genGrasps();
        //void genGraspPairs();
        void analyze(); //temp. dev purpose
        void getGraspMarker(visualization_msgs::MarkerArray &markerArray);
        //returns grasp somewhere closer to center of object. very rough approximation.
        geometry_msgs::Pose getPickUpPose();
        void pickUpTube(geometry_msgs::Pose &pickPose);
        geometry_msgs::PoseArray _work_traj; //put this back in private after dbg/dev
        geometry_msgs::PoseArray _tube_traj;
        visualization_msgs::Marker vismsg_workNormalsX;
        visualization_msgs::Marker vismsg_workNormalsY;
        visualization_msgs::Marker vismsg_workNormalsZ;
        geometry_msgs::PoseArray grasp_pose_array;

    private:
        TubePerception::Tube::Ptr _tube;
        TubeGrasp::GraspArray::Ptr _grasp_array;
        TubeGrasp::GraspPairArray::Ptr _test_pairs;
        TubeGrasp::GraspPairArray::Ptr _valid_pairs;
        unsigned long _best_pair_idx;

        geometry_msgs::Pose _work_pose;
        int _traj_idx;
        double _axis_step_size; //in mm
        int _circular_steps;  //integer number
        double _wrist_axis_offset;
        unsigned long MAX_TEST_GRASPS; //Maximum valid grasps to store
        //Maximum iteration for randomly selecting grasp to test
        unsigned long MAX_ITERATION;  //Check the repetations in selecting random index in test_for_ik_
        void _gen_grasps(double axis_step_size, int circular_steps,  GraspArray::Ptr grasp_array);
        bool _gen_work_trajectory();
        void _gen_test_pairs();
        void _normalize_worktrajectory();
        void _xform_in_tubeframe();
        void _work2tube_trajectory();
        void _test_pairs_for_ik();
        void _compute_metric();
    };
}

#endif // TUBEGRASP_H
