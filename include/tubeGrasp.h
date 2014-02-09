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
        float quality;
        unsigned int cylinderIdx;
    };
    
    class GraspPair
    {
    public:
        TubeGrasp::Grasp rightGrasp;
        TubeGrasp::Grasp leftGrasp;
        double rank;
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
        TubeGrasp::GraspPairArray::Ptr grasp_pairs_;

        //tf::Vector3 contact_vector_;
        //tf::Vector3 axis_vector_;
        geometry_msgs::Pose work_pose_;
        int traj_idx_;
        float axis_step_size_; //in mm
        int circular_steps_;  //in integer number
        float wrist_axis_offset_;
        void generate_grasps_();
        bool generate_work_trajectory_();
        void generate_grasp_pairs_();
        void normalize_worktrajectory_();
        void xform_in_tubeframe_();
        void work2tube_trajectory_();
    };
    void diaplayGraspsInGlobalFrame(TubeGrasp::GraspArray::Ptr grasp_array, tf::Transform tube_tf);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> displayGrasps(TubeGrasp::GraspArray::Ptr grasp_array);
}

#endif // TUBEGRASP_H
