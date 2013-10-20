#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/ros/conversions.h>

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

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (pcl::PointCloud<PointT>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> pointsVis (pcl::PointCloud<PointT>::ConstPtr cloud);
void estimate_normals(pcl::PointCloud<PointT>::Ptr cloud_in);
double get_radius(pcl::PointCloud<PointT>::Ptr cloud_in);
void get_largest_cluster(pcl::PointCloud<PointT>::Ptr points_in, pcl::PointCloud<PointT>::Ptr points_out);
void get_axis_points(pcl::PointCloud<PointT>::Ptr cloud_in, double r, pcl::PointCloud<PointT>::Ptr axis_points);
void process_tube_cloud(sensor_msgs::PointCloud2 &object_cloud);
void display_cloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
void display_normals(pcl::PointCloud<PointT>::Ptr cloud_with_normals);
int segmentize_axis(pcl::PointCloud<PointT>::Ptr axis_points, pcl::PointCloud<PointT>::Ptr axis_line_points);


#endif // PERCEPTION_H
