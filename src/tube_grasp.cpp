#include "tubeGrasp.h"

namespace TubeGrasp
{

Grasp::Grasp()
{
    id = 0;
    idOfPair = 0;
}

GraspAnalysis::GraspAnalysis()
{
    step_size_ = 0.02;
}

void GraspAnalysis::generateGraspsFromCylinders(std::vector<TubePerception::Cylinder> *cyl_ptr)
{
    for(size_t i=0; i<cyl_ptr->size(); )
        cyl_ptr[i].p1;
}

}// NAMESPACE TUBEGRASP
