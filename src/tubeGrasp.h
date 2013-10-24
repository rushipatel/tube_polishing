#ifndef TUBEGRASP_H
#define TUBEGRASP_H

#include <geometry_msgs/Pose.h>
#include "tubePerception.h"

namespace TubeGrasp
{
    class Grasp
    {
    public:
        Grasp();
        int id;
        int idOfPair;
        geometry_msgs::Pose pose;
        float quality;
        //TubePerception::Cylinder ofCylinder;
    };

    class GraspAnalysis
    {
    public:
        GraspAnalysis();
        std::vector<Grasp> grasps;
        generateGraspsFromCylinders(std::vector<TubePerception::Cylinder*> cyl_ptr);

    protected:
        float step_size_;
    };

}

#endif // TUBEGRASP_H
