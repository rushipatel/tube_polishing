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

#define SET_PLANNING_SCENE_DIFF_NAME "/environment_server/set_planning_scene_diff"

namespace TubeGrasp
{

    class Grasp
    {
    public:
        Grasp(){
            _wrist_offset = 0.0;
        }

        unsigned int group; //circular group. it helps reduce pairs to test
        unsigned int cylinderIdx;
        geometry_msgs::Pose getWristGlobalPose(const geometry_msgs::Pose &objectPose);
        tf::Transform getWristGlobalPose(const tf::Transform &objectTf);

        geometry_msgs::Pose getWristPose(); // computes wrist pose using _wrist_offset
        tf::Transform getWristTransform();
        geometry_msgs::Pose getWristPose(double offset); //uses given offset instead _wrist_offset
        tf::Transform getWristTransform(double offset);
        //geometry_msgs::Pose getPose(); // raw grasp pose. coinsident to cylinder axis
        void setWristOffset(const double offset);
        void setPose(const geometry_msgs::Pose &pose);
        void setPose(const tf::Transform &tf);
        tf::Transform getTransform(void);
        geometry_msgs::Pose getPose(void);
        double getWristOffset(void);
    private:
        double _wrist_offset; //offset from cylinder axis to wrist origin
        geometry_msgs::Pose _pose;

        tf::Transform _get_wrist_pose(double offset);
    };

    class GraspPair
    {
    public:
        GraspPair()
        {
            isValid = false;
            rank = 0;
        }
        TubeGrasp::Grasp rightGrasp;
        TubeGrasp::Grasp leftGrasp;
        bool isValid;
        std::vector<double> qRight; //to store ik results (joint traj) for valid_pairs
        std::vector<double> qLeft;
        //Stores minimum of two, right and left, metric
        std::vector<double> forceMetric; //Memory inefficient. Store accumulative rank only after dev/debug
        std::vector<double> rotMetric;
        double rank;
        double minForce;
        double minRot;
    };

    class GraspArray
    {
    public:
        std::vector<TubeGrasp::Grasp> grasps;
        typedef boost::shared_ptr<TubeGrasp::GraspArray> Ptr;
    };

    class GraspPairArray
    {
    public:
        std::vector<TubeGrasp::GraspPair> graspPairs;
        typedef boost::shared_ptr<TubeGrasp::GraspPairArray> Ptr;
    };

    class GraspAnalysis
    {
    public:
        GraspAnalysis(ros::NodeHandlePtr nh);
        void setTubePtr(TubePerception::Tube::Ptr tube);
        void setWorkPose(geometry_msgs::Pose &p);
        int getWorkPose(geometry_msgs::Pose &p);
        void setWorkTrajIdx(int trajIdx);
        //void genGrasps();
        //void genGraspPairs();
        void compute(); //temp. dev purpose
        void getGraspMarker(TubePerception::Tube::Ptr tube, double wrist_offset, visualization_msgs::MarkerArray &markerArray);
        //returns grasp somewhere closer to center of object. very rough approximation.
        //void pickUpTube(geometry_msgs::Pose &pickPose);
        bool getComputedGraspPair(GraspPair &graspPair);
        void getTubeWorkTrajectory(geometry_msgs::PoseArray &tube_traj);
        Grasp getPickPose(std::vector<tf::Vector3> &pointsToAvoid, double min_dist);

        geometry_msgs::PoseArray _work_traj; //put this back in private after dbg/dev
        geometry_msgs::PoseArray _tube_traj;
        visualization_msgs::Marker vismsg_workNormalsX;
        visualization_msgs::Marker vismsg_workNormalsY;
        visualization_msgs::Marker vismsg_workNormalsZ;
        geometry_msgs::PoseArray grasp_pose_array;
        typedef boost::shared_ptr<TubeGrasp::GraspAnalysis> Ptr;

    private:
        ros::NodeHandlePtr _nh;
        TubePerception::Tube::Ptr _tube;
        TubeGrasp::GraspArray::Ptr _grasp_array;
        TubeGrasp::GraspPairArray::Ptr _test_pairs;
        TubeGrasp::GraspPairArray::Ptr _valid_pairs;
        TubeGrasp::GraspPair _computed_pair;
        bool _grasp_pair_found;

        geometry_msgs::Pose _work_pose;
        int _traj_idx;
        double _axis_step_size; //in mm
        int _circular_steps;  //integer number
        double _wrist_axis_offset;
        unsigned long MAX_TEST_GRASPS; //Maximum valid grasps to store
        //Maximum iteration for randomly selecting grasp to test
        unsigned long MAX_ITERATION;  //Check the repetations in selecting random index in test_for_ik_
        void _gen_grasps(double axis_step_size, int circular_steps,  GraspArray::Ptr grasp_array, double offset);
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
