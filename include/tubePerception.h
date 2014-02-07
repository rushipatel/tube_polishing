#ifndef TUBEPERCEPTION_H
#define TUBEPERCEPTION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/ros/conversions.h>
#include <Eigen/Core>
#include <tf/LinearMath/Matrix3x3.h>

#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btQuaternion.h>
#include <tf/tf.h>

#include <pcl/sample_consensus/sac_model_circle.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/surface/mls.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/pcl_base.h>

#include <sstream>
#include "utility.cpp"

typedef pcl::PointXYZRGBNormal PointT;

namespace TubePerception
{
    class Cylinder// : public Line
    {
    public:
        PointT p1; // global
        PointT p2; // global
        //PointT centerPoint; //global
        float radius;
        bool isStrong;

        //tf::Vector3 axisVector; //Global
        tf::Vector3 getAxisVector();
        float getAxisLength();
        //std::vector<int> neighbourCylinders;
        pcl::ModelCoefficients coefficients; // global
        tf::Transform getGlobalTransform(void);
        tf::Transform getLocalTransform(void); //tf to last (strong) cylinder in vector(array)
        geometry_msgs::Pose getGlobalPose(void);
        geometry_msgs::Pose getLocalPose(void);
        //void setLocalTransform(tf::Transform &tf);
        void setGlobalPose(geometry_msgs::Pose &pose);
        void setLocalPose(geometry_msgs::Pose &pose);
        void setLocalPose(tf::Transform &t);
        tf::Vector3 getLocalAxisVector(void); // in a tube frame

    private:
        geometry_msgs::Pose global_pose_; //in global frame that is point cloud frame (base_link)

        /********* CAUTION : Orientation is Identity for local_pose************/
        geometry_msgs::Pose local_pose_;  //Local to first (strong) cylinder in vector(array)
    };
    
    /*class WorkTrajectory
    {
    public:
        geometry_msgs::PoseArray trajectory;
        tf::Vector3 pointInPlane; //point in plane
        tf::Vector3 perpToPlane; //plane perpendicular vector
        unsigned int cylinderIdx;
    };*/

    class Tube
    {
    public:
        Tube(sensor_msgs::PointCloud2 &rosTubeCloud);
        //~Tube();
        std::vector<TubePerception::Cylinder> cylinders;
        geometry_msgs::Pose getPose(void);
        void setPose(geometry_msgs::Pose &pose);
        tf::Transform getTransform();
        typedef boost::shared_ptr<TubePerception::Tube> Ptr;
        pcl::PointCloud<PointT>::Ptr tubeCloud;
        pcl::PointCloud<PointT>::Ptr axisPoints;
        //in global frame, for current state of tube. Not actual pose of tube
        //std::vector<TubePerception::WorkTrajectory> workTrajectories;
        //work trajectories by NormalArray
        std::vector<pcl::PointCloud<PointT>::Ptr> workPointsCluster;
        unsigned int whichCylinder(PointT point);
        void getCylinderMarker(visualization_msgs::MarkerArray &markerArray);
        void getCylinderPoses(geometry_msgs::PoseArray &pose_array);
    protected:
        geometry_msgs::Pose pose_;  //in global(base_link) frame
    };

    class CloudProcessing
    {
    public:

        CloudProcessing(TubePerception::Tube::Ptr tube_ptr)
        {
            tube_ = tube_ptr;
            num_of_points_ = tube_->tubeCloud->points.size();
            r_ = 0;
            strong_line_thr_ = 0.2;
            weak_line_thr_ = 0.1;
            min_points_ = 0.05;
            z_error_ = 0;

            processCloud_();
        }
        //CloudProcessing(sensor_msgs::PointCloud2 &tubeCloud, geometry_msgs::Pose sensorPose);
        //~CloudProcessing();

        void displayCloud(void);
        void displayAxisPoints(void);
        void displayCylinders(void);
        void displayCylinders(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
        void displayLines(void);
        void displayCylindersInLocalFrame(void);
        void setZerror(float error);
        bool writeAxisPointsOnFile(std::string fileName);
        void dispalyWorkTraj(void);

    private:
        void processCloud_(void);
        void estimate_normals_(void);
        void get_radius_(void);
        void collaps_normals_(void);
        bool find_line_(pcl::PointIndices::Ptr inliers, Cylinder *cyl);
        void remove_inliers_(pcl::PointCloud<PointT>::Ptr points, pcl::PointIndices::Ptr indices);
        void remove_inliers_(pcl::PointCloud<PointT>::Ptr points,  std::vector<int> &indices);
        void get_line_points_(pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients line_coeff, PointT &p1, PointT &p2);
        void segmentize_axis_(void);
        void compensate_error_(void);
        void cylinder_filter_(Cylinder cyl, pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointIndices::Ptr inliers);
        float is_in_cylinder_( const PointT & pt1, const PointT & pt2, float length_sq, float radius_sq, const PointT & testpt );
        void get_line_graph_(void);
        void print_line_graph_(void);
        void add_neighbour_(int cyl_ind, int neighbour_ind);
        tf::Vector3 get_perp_vec3_(tf::Vector3 v3);
        void define_pose_(void);
        void define_pose2_(void);
        void generate_work_vectors_();
        float r_;
        float strong_line_thr_;
        float min_points_;
        float weak_line_thr_;
        pcl::PointCloud<PointT>::Ptr raw_axis_points_;
        float z_error_;
        int num_of_points_;
        TubePerception::Tube::Ptr tube_;
    };

}
#endif // TUBEPERCEPTION_H
