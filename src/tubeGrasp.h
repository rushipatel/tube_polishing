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
        //TubePerception::Cylinder ofCylinder;
    };

    class GraspArray
    {
    public:
        std::vector<TubeGrasp::Grasp> grasps;
        typedef boost::shared_ptr<TubeGrasp::GraspArray> Ptr;
    };

    class GraspAnalysis
    {
    public:
        GraspAnalysis(TubeGrasp::GraspArray::Ptr grasp_array);
        void generateGrasps(TubePerception::Tube::Ptr tube);

    private:
        TubeGrasp::GraspArray::Ptr grasp_array_;
        float axis_step_size_;
        int circular_steps_;
        float wrist_axis_offset_;
    };
    void diaplayGraspsInGlobalFrame(TubeGrasp::GraspArray::Ptr grasp_array, tf::Transform tube_tf);
    void displayGrasps(TubeGrasp::GraspArray::Ptr grasp_array);
}

#endif // TUBEGRASP_H
