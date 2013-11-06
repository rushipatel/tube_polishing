#ifndef TUBEPERCEPTION_H
#define TUBEPERCEPTION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Pose.h>
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
#include <pcl/visualization/cloud_viewer.h>
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

typedef pcl::PointXYZRGBNormal PointT;

namespace TubePerception
{
    class Cylinder// : public Line
    {
    public:
        PointT p1;
        PointT p2;
        PointT centerPoint;
        float radius;
        bool isStrong;

        tf::Vector3 axisVector;
        std::vector<int> neighbourCylinders;
        pcl::ModelCoefficients coefficients;
        tf::Transform getGlobalTransform(void);
        tf::Transform getLocalTransform(void); //tf to first cylinder in vector
        geometry_msgs::Pose getGlobalPose(void);
        geometry_msgs::Pose getLocalPose(void);
        void setLocalTransform(tf::Transform &tf);
        void setGlobalPose(geometry_msgs::Pose &pose);

    private:
        geometry_msgs::Pose pose_; //global to frame that is point cloud frame (base_link)
        tf::Transform local_tf_; //Local to first cylinder
    };

    class Tube
    {
    public:
        Tube(sensor_msgs::PointCloud2 &rosTubeCloud);
        //~Tube();
        std::vector<TubePerception::Cylinder> cylinders;
        geometry_msgs::Pose getPose(void);
        tf::Transform getTransform(void);
        void setPose(geometry_msgs::Pose &pose);
        typedef boost::shared_ptr<TubePerception::Tube> Ptr;
        pcl::PointCloud<PointT>::Ptr tubeCloud;
        pcl::PointCloud<PointT>::Ptr axisPoints;

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
        void displayLines(void);
        void displayCylindersInLocalFrame(void);
        void setZerror(float error);
        bool writeAxisPointsOnFile(std::string fileName);

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
