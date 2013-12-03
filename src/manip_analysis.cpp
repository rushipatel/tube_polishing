#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/segment.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <tf/tf.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

//typedef boost::shared_ptr<std::vector<double>> JointValuePtr;

class manipAnalysis
{
public:
    manipAnalysis(std::string which_arm, const ros::NodeHandle &nh);
    void setForceVec(tf::Vector3 vec);
    void setRotationAxis(tf::Vector3 vec);
    void setReferencePoint(tf::Vector3 pInTipFrame);
    double getForceMatric(const std::vector<double> &q);
    double getRotationMatric(const std::vector<double> &q);
    double getManipIndex(const std::vector<double> &q);
    
private:
    std::vector<double> q_;;
    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd jacobian_t_;  //3x6 top 3 rows
    Eigen::MatrixXd jacobian_r_;  //3x6 bottom 3 rows
    Eigen::Vector3d force_vec_;
    Eigen::Vector3d axis_vec_;
    Eigen::Vector3d ref_point_;
    Eigen::MatrixXd sigma_t_;
    Eigen::MatrixXd sigma_r_;
    KDL::Chain chain_;
    
    void update_jacobian_(void);
    double find_intersecting_len_(bool force);
    double get_manipulability_index_(void);
};

manipAnalysis::manipAnalysis(std::string which_arm, const ros::NodeHandle &nh)
{
    bool right_arm;
    if(which_arm.compare("right_arm"))
        right_arm = true;
    else if(which_arm.compare("left_arm"))
        right_arm = false;
    else
    {
        ROS_ERROR("manip_analysis: Please specify 'right_arm' or 'left_arm'. Using right_arm as default");
        right_arm = true;
    }
    KDL::Tree pr2;
    /*if (!kdl_parser::treeFromFile("../pr2.urdf", pr2))
    {
       ROS_ERROR("Failed to construct kdl tree from file");
    }*/
    std::string robot_desc;
    nh.getParam("robot_description", robot_desc);
    if (!kdl_parser::treeFromString(robot_desc,pr2))
    {
        ROS_ERROR("Failed to construct kdl tree from parameter server");
    }
    
    if(right_arm)
        pr2.getChain("base_footprint","r_wrist_roll_link",chain_);
    else
        pr2.getChain("base_footprint","l_wrist_roll_link",chain_);
    
    /*q_default_.resize(7);
    if(right_arm)
    {
        q_default_[0] = -1.5;
        q_default_[1] =  0.0;
        q_default_[2] =  0.0;
        q_default_[3] =  0.0;
        q_default_[4] = -0.15;
        q_default_[5] = -0.1;
        q_default_[6] =  0.0;
    }
    else
    {
        q_default_[0] = -1.5;
        q_default_[1] =  0.0;
        q_default_[2] =  0.0;
        q_default_[3] =  0.0;
        q_default_[4] = -0.15;
        q_default_[5] = -0.1;
        q_default_[6] =  0.0;
    }*/
    ref_point_(0) = 0.0;
    ref_point_(1) = 0.0;
    ref_point_(2) = 0.0;
}

void manipAnalysis::update_jacobian_(void)
{
    KDL::Jacobian jacob(chain_.getNrOfJoints());
    KDL::JntArray q_in(chain_.getNrOfJoints());
    
    
    if(q_.empty() || q_.size()!=7)
    {
        ROS_ERROR("maip_analysis - Joint values are not given or size is not 7.");
    }
    
    q_in(0) = 0.0; //Torso Lift joint
    for(int i=0; i<7; i++)
        q_in(i+1) = q_[i];
    
    KDL::ChainJntToJacSolver jac_solver(chain_);
    jac_solver.JntToJac(q_in,jacob);
    
    jacobian_ = jacob.data.block(0,1,6,7); //discard first row related to torso lift
    jacobian_t_ = jacobian_.topRows(3);
    jacobian_r_ = jacobian_.bottomRows(3);
}

double manipAnalysis::getForceMatric(const std::vector<double> &q)
{
    q_=q;
    update_jacobian_();
    q_.clear();
    return find_intersecting_len_(true);
    
}

double manipAnalysis::getRotationMatric(const std::vector<double> &q)
{
    q_=q;
    update_jacobian_();
    q_.clear();
    return find_intersecting_len_(false);
}

double manipAnalysis::getManipIndex(const std::vector<double> &q)
{
    q_ = q;
    update_jacobian_();
    q_.clear();
    return get_manipulability_index_();
}

double manipAnalysis::get_manipulability_index_()
{
    Eigen::MatrixXd matrix = jacobian_ * jacobian_.transpose();
    Eigen::MatrixXd eigen_values;
    Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(matrix);
    eigen_values = eigensolver.eigenvalues().real();
    ROS_INFO_STREAM("Eigen values : "<<eigen_values);
    return (eigen_values.minCoeff()/eigen_values.maxCoeff());
}

double manipAnalysis::find_intersecting_len_(bool force)
{
    Eigen::MatrixXd matrix;
    if(force)
        matrix = jacobian_t_ * jacobian_t_.transpose();
    else
        matrix = jacobian_r_ * jacobian_r_.transpose();
    
    Eigen::MatrixXcd eigen_values;
    Eigen::MatrixXcd eigen_vectors;
    Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(matrix);
    eigen_values = eigensolver.eigenvalues();
    eigen_vectors = eigensolver.eigenvectors();
    //Vector in Global frame around which we want rotational manipulability ellipsoid intersection
    //or force ellipsoid intersection
    Eigen::Vector3d vec;
    if(force)
        vec = force_vec_;
    else
        vec = axis_vec_;
                
    Eigen::Vector3d vec_e; //vec in ellipsoid frame
    Eigen::MatrixXd A = eigen_vectors.real();
    vec_e = A.transpose() * vec;
    double l, l_den;
  
    Eigen::MatrixXd sigma = eigen_values.real();
    double s1,s2,s3;
    if(force)  //invert sigmas for force measure
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
  
    l_den = ((vec_e[0]*vec_e[0])/(s1*s1)) + 
            ((vec_e[1]*vec_e[1])/(s2*s2)) + 
            ((vec_e[2]*vec_e[2])/(s3*s3)) ;
    l = sqrt(1/l_den);
  
    return l;
}

void manipAnalysis::setForceVec(tf::Vector3 vec)
{
    force_vec_(0) = vec.x();
    force_vec_(1) = vec.y();
    force_vec_(2) = vec.z();
}

void manipAnalysis::setRotationAxis(tf::Vector3 vec)
{
    axis_vec_(0) = vec.x();
    axis_vec_(1) = vec.y();
    axis_vec_(2) = vec.z();
}

void manipAnalysis::setReferencePoint(tf::Vector3 inTipFrame)
{
    ref_point_(0) = inTipFrame.x();
    ref_point_(1) = inTipFrame.y();
    ref_point_(2) = inTipFrame.z();
}
