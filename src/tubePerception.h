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

typedef pcl::PointXYZRGBNormal PointT;

class tubePerception
{   
public:
    struct line
    {
        PointT p1;
        PointT p2;
        //float l;
        //float m;
    };
    tubePerception(sensor_msgs::PointCloud2 &tubeCloud);
    tubePerception(sensor_msgs::PointCloud2 &tubeCloud, geometry_msgs::Pose sensorPose);
    ~tubePerception();
    void processCloud(void);
    std::vector<line> lines;

private:
    //sensor_msgs::PointCloud2  ros_tube_cloud;
    pcl::PointCloud<PointT>::Ptr tube_cloud_;
    pcl::PointCloud<PointT>::Ptr axis_points_;
    pcl::PointCloud<PointT>::Ptr raw_axis_points_;
    void estimate_normals_(void);
    void get_radius_(void);
    void collaps_normals_(void);
    float r_; //std::vector<float> r_
    const int strong_line_points_ = 100;
    const int weak_line_points_ = 10;
};

#endif // TUBEPERCEPTION_H
