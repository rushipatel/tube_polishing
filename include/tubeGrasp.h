#ifndef TUBEGRASP_H
#define TUBEGRASP_H

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

#include "tubePerception.h"
#include "manip_analysis.cpp"
#include "utility.cpp"
#include "dualArms.h"

namespace TubeGrasp
{
    class Grasp
    {
    public:
        Grasp();
        geometry_msgs::Pose wristPose;
        unsigned int group; //circular group. it helps reduce pairs to test
        //unsigned int cylinderIdx;
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
        ros::NodeHandle nodeHandle;
        GraspAnalysis(TubePerception::Tube::Ptr tube, ros::NodeHandle nh);
        void setWorkPose(geometry_msgs::Pose &p);
        int getWorkPose(geometry_msgs::Pose &p);
        void setContactVector(tf::Vector3 contactVector, tf::Vector3 axisVector);
        void setWorkTrajIdx(int trajIdx);
        void generateGrasps();
        void generateGraspPairs();
        void generateWorkTrajectory(); //temp. dev purpose
        void getGraspMarker(visualization_msgs::MarkerArray &markerArray);
        geometry_msgs::PoseArray work_traj_; //put this back in private after dbg/dev
        geometry_msgs::PoseArray tube_traj_;
        visualization_msgs::Marker vismsg_workNormalsX;
        visualization_msgs::Marker vismsg_workNormalsY;
        visualization_msgs::Marker vismsg_workNormalsZ;
        geometry_msgs::PoseArray grasp_pose_array;

    private:
        TubePerception::Tube::Ptr tube_;
        TubeGrasp::GraspArray::Ptr grasp_array_;
        TubeGrasp::GraspPairArray::Ptr test_pairs_;
        TubeGrasp::GraspPairArray::Ptr valid_pairs_;

        //tf::Vector3 contact_vector_;
        //tf::Vector3 axis_vector_;
        geometry_msgs::Pose work_pose_;
        int traj_idx_;
        float axis_step_size_; //in mm
        int circular_steps_;  //in integer number
        float wrist_axis_offset_;
        unsigned long MAX_TEST_GRASPS; //Maximum valid grasps to store
        //Maximum iteration for randomly selecting grasp to test
        unsigned long MAX_ITERATION;  //Check the repetations in selecting random index in test_for_ik_
        void generate_grasps_();
        bool generate_work_trajectory_();
        void generate_test_pairs_();
        void normalize_worktrajectory_();
        void xform_in_tubeframe_();
        void work2tube_trajectory_();
        void test_pairs_for_ik_();
        void compute_metric_();
    };
    void diaplayGraspsInGlobalFrame(TubeGrasp::GraspArray::Ptr grasp_array, tf::Transform tube_tf);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> displayGrasps(TubeGrasp::GraspArray::Ptr grasp_array);
}

#endif // TUBEGRASP_H
