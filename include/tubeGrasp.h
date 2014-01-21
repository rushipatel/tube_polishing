#ifndef TUBEGRASP_H
#define TUBEGRASP_H

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

#include "tubePerception.h"

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
        GraspAnalysis(TubePerception::Tube::Ptr tube);
        void setWorkPose(geometry_msgs::Pose pose);
        void setContactVector(tf::Vector3 contactVector, tf::Vector3 axisVector);
        void setWorkTrajIdx(int trajIdx);
        void generateGrasps();
        void generateGraspPairs();
        void generateWorkTrajectory(); //temp. dev purpose
        geometry_msgs::PoseArray trajectory_; //put this back in private after dbg/dev

    private:
        TubePerception::Tube::Ptr tube_;
        TubeGrasp::GraspArray::Ptr grasp_array_;
        TubeGrasp::GraspPairArray::Ptr grasp_pairs_;

        //tf::Vector3 contact_vector_;
        //tf::Vector3 axis_vector_;
        geometry_msgs::Pose work_pose_;
        int traj_idx_;
        float axis_step_size_;
        int circular_steps_;
        float wrist_axis_offset_;
        void generate_grasps_();
        bool generate_work_trajectory_();
        void generate_grasp_pairs_();
    };
    void diaplayGraspsInGlobalFrame(TubeGrasp::GraspArray::Ptr grasp_array, tf::Transform tube_tf);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> displayGrasps(TubeGrasp::GraspArray::Ptr grasp_array);
}

#endif // TUBEGRASP_H
