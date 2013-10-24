#ifndef TUBEPERCEPTION_H
#define TUBEPERCEPTION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Pose.h>
#include <pcl/ros/conversions.h>
#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/common/io.h>
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

#include <pcl/sample_consensus/sac_model_circle.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/surface/mls.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <boost/thread/thread.hpp>
#include <sstream>

typedef pcl::PointXYZRGBNormal PointT;

namespace TubePerception
{
    class Cylinder// : public Line
    {
    public:
        PointT p1;
        PointT p2;
        float radius;
        bool isStrong;
        pcl::ModelCoefficients coefficients;
    };

    class Tube
    {
    public:
        Tube(sensor_msgs::PointCloud2 &tubeCloud);
        //~Tube();
       std::vector<TubePerception::Cylinder> cylinders;
       typedef boost::shared_ptr<Tube> Ptr;

    protected:
        pcl::PointCloud<PointT>::Ptr tube_cloud_;
        pcl::PointCloud<PointT>::Ptr axis_points_;
        int num_of_points_;
    };

    class CloudProcessing : public Tube
    {
    public:
        CloudProcessing(sensor_msgs::PointCloud2 &tubeCloud) : Tube(tubeCloud)
        {
            num_of_points_ = tube_cloud_->points.size();
            r_ = 0;
            strong_line_thr_ = 0.1;
            weak_line_thr_ = 0.02;
            z_error_ = 0;
        }
        //CloudProcessing(sensor_msgs::PointCloud2 &tubeCloud, geometry_msgs::Pose sensorPose);
        //~CloudProcessing();
        void processCloud(void);
        void displayCloud(int sec);
        void displayAxisPoints(int sec);
        void displayCylinders(int sec);
        void displayLines(int sec);
        void setZerror(float error);

    private:
        void estimate_normals_(void);
        void get_radius_(void);
        void collaps_normals_(void);
        bool find_line_(pcl::PointIndices::Ptr inliers, Cylinder *cyl);
        void remove_inliers_(pcl::PointCloud<PointT>::Ptr points, pcl::PointIndices::Ptr indices);
        void remove_inliers_(pcl::PointCloud<PointT>::Ptr points,  std::vector<int> &indices);
        void get_line_points_(pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients line_coeff, Cylinder* cylinder);
        void segmentize_axis_(void);
        void compensateError(void);
        void cylinder_filter_(Cylinder cyl, pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointIndices::Ptr inliers);
        float is_in_cylinder_( const PointT & pt1, const PointT & pt2, float length_sq, float radius_sq, const PointT & testpt );
        float r_;
        float strong_line_thr_;
        float weak_line_thr_;
        pcl::PointCloud<PointT>::Ptr raw_axis_points_;
        float z_error_;
    };

}
#endif // TUBEPERCEPTION_H
