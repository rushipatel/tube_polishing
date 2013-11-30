#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/segment.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

double find_intersecting_len(Eigen::MatrixXd jacobian, Eigen::Vector3d vec,bool invertSigma=false)
{
    Eigen::MatrixXd matrix = jacobian * jacobian.transpose();
    Eigen::MatrixXcd eigen_values;
    Eigen::MatrixXcd eigen_vectors;
    Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(matrix);
    eigen_values = eigensolver.eigenvalues();
    eigen_vectors = eigensolver.eigenvectors();
      
    //Vector in Global frame around which we want rotational manipulability ellipsoid intersection
    vec.setZero();
    vec[0] = 1;
    Eigen::Vector3d vec_e; //vec in ellipsoid frame
    Eigen::MatrixXd A = eigen_vectors.real();
    vec_e = A.transpose() * vec;
    double l, l_den;
    double xt = vec_e[0];
    double yt = vec_e[1];
    double zt = vec_e[2];
  
    Eigen::MatrixXd sigma = eigen_values.real();
    double s1,s2,s3;
    if(invertSigma)  //invert sigmas for force measure
    {
        s1 = 1/sigma(0);
        s2 = 1/sigma(1);
        s3 = 1/sigma(2);
    }
    else
    {
        s1 = sigma(0);
        s2 = sigma(1);
        s3 = sigma(2);
    }
  
    l_den = ((xt*xt)/(s1*s1)) + ((yt*yt)/(s2*s2)) + ((zt*zt)/(s3*s3));
    l = sqrt(1/l_den);
  
    return l;
}

void get_jacobian(bool right_arm, ros::NodeHandle nh)
{
    KDL::Tree pr2;
    if (!kdl_parser::treeFromFile("../pr2.urdf", pr2))
    {
       ROS_ERROR("Failed to construct kdl tree from file");
       //return;
    }
    std::string robot_desc;
    nh.getParam("robot_description", robot_desc);
    if (!kdl_parser::treeFromString(robot_desc,pr2))
    {
        ROS_ERROR("Failed to construct kdl tree from parameter server");
    }
    
    KDL::Chain chain;
    
    if(right_arm)
        pr2.getChain("base_footprint","r_wrist_roll_link",chain);
    else
        pr2.getChain("base_footprint","l_wrist_roll_link",chain);
    KDL::Segment seg;
    std::vector<KDL::Segment> segs;
    ROS_INFO_STREAM("Nr of segments: "<<chain.getNrOfSegments());
    ROS_INFO_STREAM("Nr of Joints: "<<chain.getNrOfJoints());
    
    for(int i=0; i<chain.segments.size(); i++)
    {
        seg = chain.segments[i];
        ROS_INFO_STREAM("Selected Segment "<<i<<" : "<<seg.getName());
    }
    KDL::Joint j;
    for(int i=0; i<chain.segments.size(); i++)
    {
        seg = chain.segments[i];
        j = seg.getJoint();
        ROS_INFO_STREAM("Selected Joint "<<i<<" : "<<j.getName());
    }
    
    KDL::Jacobian jacob(chain.getNrOfJoints());
    KDL::JntArray q_in(chain.getNrOfJoints());
    
    q_in(0) = 0.0;
    q_in(1) = -1.5;
    q_in(2) = 0.0;
    q_in(3) = 0.0;
    q_in(4) = 0.0;
    q_in(5) = -0.15;
    q_in(6) = -0.1;
    q_in(7) = 0.0;
    
    KDL::ChainJntToJacSolver jac_solver(chain);
    jac_solver.JntToJac(q_in,jacob);
    
    Eigen::Matrix<double,6,7> jacob2 = jacob.data.block(0,1,6,7);
    ROS_INFO_STREAM("J2: "<<"\n"<<jacob2);
    
    Eigen::MatrixXd jacobian_t = jacob2.topRows(3);
    Eigen::MatrixXd jacobian_r = jacob2.bottomRows(3);
    
    double manip = find_intersecting_len(jacobian_r, Eigen::Vector3d(0,1,0), false);
    double force = find_intersecting_len(jacobian_t, Eigen::Vector3d(1,0,0), true);
    
    ROS_INFO_STREAM("Manip : "<<manip<<"     Force : "<<force);
    
}

/*void get_jacobian(void)
{
    KDL::Tree pr2;
    if (!kdl_parser::treeFromFile("../pr2.urdf", pr2))
    {
       ROS_ERROR("Failed to construct kdl tree");
       return;
    }
    KDL::Chain chain;
    
    pr2.getChain("base_footprint","r_wrist_roll_link",chain);
    
    KDL::Segment seg;
    std::vector<KDL::Segment> segs;
    ROS_INFO_STREAM("Nr of segments: "<<chain.getNrOfSegments());
    ROS_INFO_STREAM("Nr of Joints: "<<chain.getNrOfJoints());
    
    for(int i=0; i<chain.segments.size(); i++)
    {
        seg = chain.segments[i];
        ROS_INFO_STREAM("Selected Segment "<<i<<" : "<<seg.getName());
    }
    KDL::Joint j;
    for(int i=0; i<chain.segments.size(); i++)
    {
        seg = chain.segments[i];
        j = seg.getJoint();
        ROS_INFO_STREAM("Selected Joint "<<i<<" : "<<j.getName());
    }
    
    KDL::Jacobian jacob(chain.getNrOfJoints());
    KDL::JntArray q_in(chain.getNrOfJoints());
    
    q_in(0) = 0.0;
    q_in(1) = -1.5;
    q_in(2) = 0.0;
    q_in(3) = 0.0;
    q_in(4) = 0.0;
    q_in(5) = -0.15;
    q_in(6) = -0.1;
    q_in(7) = 0.0;
    
    std::vector<bool> locked_joints;
    for(int i=0; i<11; i++)
      locked_joints.push_back(false);
    locked_joints[0] = true;
    locked_joints[1] = true;
    locked_joints[5] = true;
    locked_joints[8] = true;
    
    KDL::ChainJntToJacSolver jac_solver(chain);
    jac_solver.JntToJac(q_in,jacob);
    
    ROS_INFO_STREAM("Jacob Colmns:"<<jacob.columns()<<"  rows:"<<jacob.rows());
    ROS_INFO_STREAM("J: "<<"\n"<<jacob.data);
    Eigen::Matrix<double,6,7> jacob2 = jacob.data.block(0,1,6,7);
    ROS_INFO_STREAM("J2: "<<"\n"<<jacob2);
}
*/
