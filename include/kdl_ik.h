#ifndef KDL_IK_H
#define KDL_IK_H

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <tf/tf.h>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

#include "utility.h"

//typedef boost::shared_ptr<std::vector<double>> JointValuePtr;

class kdl_ik
{
public:
    kdl_ik(const ros::NodeHandlePtr nh);
    bool getRightIK(const geometry_msgs::Pose &goalPose, std::vector<double> &qOut, std::vector<double> &qIn);
    bool getLeftIK(const geometry_msgs::Pose &goalPose, std::vector<double> &qOut, std::vector<double> &qIn);
    void printChainInfo();
    void setWrldToThisRootTF(tf::Transform &tf);

private:
    KDL::Chain _right_chain;
    KDL::Chain _left_chain;
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> _r_fkp;
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> _l_fkp;
    boost::shared_ptr<KDL::ChainIkSolverVel_pinv> _r_ikv;
    boost::shared_ptr<KDL::ChainIkSolverVel_pinv> _l_ikv;
    boost::shared_ptr<KDL::ChainIkSolverPos_NR> _r_ikp;
    boost::shared_ptr<KDL::ChainIkSolverPos_NR> _l_ikp;

    KDL::JntArray _r_jnts;
    KDL::JntArray _l_jnts;

    tf::Transform _world_to_this_root_link;

    void _convert_from_world_to_root(const geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out);
};

kdl_ik::kdl_ik(const ros::NodeHandlePtr nh)
{
    KDL::Tree pr2;
    /*if (!kdl_parser::treeFromFile("../pr2.urdf", pr2)){
       ROS_ERROR("Failed to construct kdl tree from file");
    }*/
    std::string robot_desc;
    nh->getParam("robot_description", robot_desc);
    if (!kdl_parser::treeFromString(robot_desc,pr2)){
        ROS_ERROR("Failed to construct kdl tree from parameter server");
        return;
    }

    pr2.getChain("torso_lift_link","r_wrist_roll_link",_right_chain);
    pr2.getChain("torso_lift_link","l_wrist_roll_link",_left_chain);

    _r_fkp.reset(new KDL::ChainFkSolverPos_recursive(_right_chain));
    _r_ikv.reset(new KDL::ChainIkSolverVel_pinv(_right_chain));
    _r_ikp.reset(new KDL::ChainIkSolverPos_NR(_right_chain, *_r_fkp, *_r_ikv, 200));

    _l_fkp.reset(new KDL::ChainFkSolverPos_recursive(_left_chain));
    _l_ikv.reset(new KDL::ChainIkSolverVel_pinv(_left_chain));
    _l_ikp.reset(new KDL::ChainIkSolverPos_NR(_left_chain, *_l_fkp, *_l_ikv, 200));

    _world_to_this_root_link.setIdentity();
    _world_to_this_root_link.setOrigin(tf::Vector3(-0.05,0.0,0.739675)); //base_link to torso_lift_link
    ROS_WARN_ONCE("kdl_ik - Root link is torso_lift_link for ik computation. Default transform is set from base_link to torso_lift_link assuming torso_lift_joint is 0 to convert incoming pose from base_link to root link.");
}

void kdl_ik::setWrldToThisRootTF(tf::Transform &tf){
    _world_to_this_root_link = tf;
}


bool kdl_ik::getRightIK(const geometry_msgs::Pose &goalPose, std::vector<double> &qOut, std::vector<double> &qIn)
{
    geometry_msgs::Pose pose;
    _convert_from_world_to_root(goalPose, pose);

    if(qIn.size()!=_right_chain.getNrOfJoints()){
        ROS_ERROR("kdl_ik - Number of in joints is not matching with joints(%d) in right chain",_right_chain.getNrOfJoints());
        return false;
    }
    qOut.resize(qIn.size());

    KDL::JntArray q_in(qIn.size());
    KDL::JntArray q_out(qIn.size());
    for(unsigned int i=0; i<qIn.size(); i++){
        q_in(i) = qIn[i];
    }
    KDL::Frame frame;
    frame.p[0] = pose.position.x;
    frame.p[1] = pose.position.y;
    frame.p[2] = pose.position.z;
    frame.M = KDL::Rotation::Quaternion( pose.orientation.x,
                                         pose.orientation.y,
                                         pose.orientation.z,
                                         pose.orientation.w);

    if(!(_r_ikp->CartToJnt(q_in, frame, q_out)<0)){
        for(unsigned int i=0; i<qOut.size(); i++){
            qOut[i] = q_out(i);
        }
        return true;
    }
    else
        return false;
}

bool kdl_ik::getLeftIK(const geometry_msgs::Pose &goalPose, std::vector<double> &qOut, std::vector<double> &qIn)
{
    geometry_msgs::Pose pose;
    _convert_from_world_to_root(goalPose, pose);

    if(qIn.size()!=_left_chain.getNrOfJoints()){
        ROS_ERROR("kdl_ik - Number of in joints is not matching with joints(%d) in right chain",_left_chain.getNrOfJoints());
        return false;
    }
    qOut.resize(qIn.size());

    KDL::JntArray q_in(qIn.size());
    KDL::JntArray q_out(qIn.size());
    for(unsigned int i=0; i<qIn.size(); i++){
        q_in(i) = qIn[i];
    }
    KDL::Frame frame;
    frame.p[0] = pose.position.x;
    frame.p[1] = pose.position.y;
    frame.p[2] = pose.position.z;
    frame.M = KDL::Rotation::Quaternion( pose.orientation.x,
                                         pose.orientation.y,
                                         pose.orientation.z,
                                         pose.orientation.w);
    if(!(_l_ikp->CartToJnt(q_in, frame, q_out)<0)){ //not less than zero. refer KDL documentation
        for(unsigned int i=0; i<qOut.size(); i++){
            qOut[i] = q_out(i);
        }
        return true;
    }
    return false;
}

void kdl_ik::_convert_from_world_to_root(const geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out)
{
    tf::Transform in = pose2tf(pose_in), out;
    out = _world_to_this_root_link.inverseTimes(in);
    pose_out = tf2pose(out);
}

void kdl_ik::printChainInfo()
{
    std::cout<<std::endl<<"Right Chain:"<<std::endl;
    std::cout<<"\tNumber of Joints   : "<<_right_chain.getNrOfJoints()<<std::endl;
    std::cout<<"\tNumber of Segments : "<<_right_chain.getNrOfSegments()<<std::endl;
    for(unsigned int i=0; i<_right_chain.getNrOfSegments(); i++){
        KDL::Segment seg(_right_chain.getSegment(i));
        KDL::Joint jnt = seg.getJoint();
        std::cout<<"\t\tSegment : "<<seg.getName()<<"\tJoint: "<<jnt.getName()<<std::endl;
    }
    std::cout<<std::endl<<"Left Chain:"<<std::endl;
    std::cout<<"\tNumber of Joints   : "<<_left_chain.getNrOfJoints()<<std::endl;
    std::cout<<"\tNumber of Segments : "<<_left_chain.getNrOfSegments()<<std::endl;
    for(unsigned int i=0; i<_left_chain.getNrOfSegments(); i++){
        KDL::Segment seg(_left_chain.getSegment(i));
        KDL::Joint jnt = seg.getJoint();
        std::cout<<"\t\tSegment : "<<seg.getName()<<"\tJoint: "<<jnt.getName()<<std::endl;
    }
    //std::cout<<"\tNote: First Joint i.e. torso_lift_joint is always zero"<<std::endl;
}

#endif // KDL_IK_H
