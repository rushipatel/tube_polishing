#ifndef GRASP_H
#define GRASP_H

#include <vector>
#include <boost/shared_ptr.hpp>

namespace TubeGrasp
{
class Grasp
{
public:
    Grasp(){}
    geometry_msgs::Pose wristPose;
    unsigned int group; //circular group. it helps reduce pairs to test
    unsigned int cylinderIdx;
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
}

#endif
