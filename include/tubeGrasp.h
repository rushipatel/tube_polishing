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
        GraspAnalysis(TubeGrasp::GraspArray::Ptr grasp_array);
        void generateGrasps(TubePerception::Tube::Ptr tube);
        void generateGraspPairs(void);

    private:
        TubeGrasp::GraspArray::Ptr grasp_array_;
        TubeGrasp::GraspPairArray::Ptr grasp_pair_array_;
        float axis_step_size_;
        int circular_steps_;
        float wrist_axis_offset_;
    };
    void diaplayGraspsInGlobalFrame(TubeGrasp::GraspArray::Ptr grasp_array, tf::Transform tube_tf);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> displayGrasps(TubeGrasp::GraspArray::Ptr grasp_array);
}

#endif // TUBEGRASP_H
