#ifndef TUBEPERCEPTION_H
#define TUBEPERCEPTION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
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
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_ros/transforms.h>

#include <LinearMath/btVector3.h>
#include <LinearMath/btQuaternion.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>

#include <pcl/sample_consensus/sac_model_circle.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/surface/mls.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/pcl_base.h>

#include <sstream>
#include "utility.h"

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

        tf::Vector3 getAxisVector();
        float getAxisLength();
        pcl::ModelCoefficients coefficients; // global
        tf::Transform getTransform(void); //tf to last (strong) cylinder in vector(array)
        geometry_msgs::Pose getPose(void);
        //tf::Vector3 getLocalAxisVector(void); // in a tube frame
        void setPose(const geometry_msgs::Pose &pose);
        void setPose(const tf::Transform &t);
        void getMarkers(visualization_msgs::MarkerArray &markerArray);
        tf::Vector3 getMidPoint(void);

    private:
        geometry_msgs::Pose _local_pose;  //Local to first (strong) cylinder in vector(array)
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
        Tube();
        //~Tube();
        std::vector<TubePerception::Cylinder> cylinders;
        geometry_msgs::Pose getPose(void);
        void setPose(geometry_msgs::Pose &pose);
        void setPoseAsActualPose(); //copies pose in to actual pose
        geometry_msgs::Pose getActualPose();
        void resetActualPose(geometry_msgs::Pose &graspPose,
                             geometry_msgs::Pose &wristPose);
        tf::Transform getTransform();
        void reset(void);
        typedef boost::shared_ptr<TubePerception::Tube> Ptr;

        //in global frame, for current state of tube. Not actual pose of tube
        //std::vector<TubePerception::WorkTrajectory> workTrajectories;
        //work trajectories by NormalArray

        std::vector<pcl::PointCloud<PointT>::Ptr> workPointsCluster;
        unsigned int whichCylinder(PointT point);
        geometry_msgs::Pose getCylinderGlobalPose(unsigned int cylIdx);
        void getCylinderMarker(visualization_msgs::MarkerArray &markerArray);
        void getCylinderPoses(geometry_msgs::PoseArray &pose_array);

        arm_navigation_msgs::AttachedCollisionObject
        getAttachedObjForRightGrasp(geometry_msgs::Pose &right_grasp_pose);

        arm_navigation_msgs::AttachedCollisionObject
        getAttachedObjForLeftGrasp(geometry_msgs::Pose &left_grasp_pose);

        arm_navigation_msgs::AttachedCollisionObject
        getAttachedObjForBothGrasps(geometry_msgs::Pose &right_grasp_pose);

    protected:
        geometry_msgs::Pose _pose;  //in global(base_link) frame.
        geometry_msgs::Pose _actual_pose; //in global(base_link)
        void _get_attached_collision_object(arm_navigation_msgs::AttachedCollisionObject &obj,
                                            geometry_msgs::Pose &grasp_pose,
                                            std::string link_name,
                                            bool right_side, bool left_side);
    };

    class CloudProcessing
    {
    public:

        CloudProcessing();
        //~CloudProcessing();

        void displayCloud(pcl::PointCloud<PointT>::Ptr cloud);
        /*void displayAxisPoints(void);
        void displayCylinders(void);
        void displayCylinders(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
        void displayLines(void);
        void displayCylindersInLocalFrame(void);*/
        void setZerror(float error);
        bool writePointCloudOnfile(const sensor_msgs::PointCloud2 &rosCloud,
                                   std::string fileName);
        bool readPointCloud(std::string fileName,
                            pcl::PointCloud<PointT>::Ptr cloudOut);
        bool writeAxisPointsOnFile(std::string fileName);
        void dispalyWorkTraj(void);
        void genTubeModel(const sensor_msgs::PointCloud2 &clusterCloud,
                          Tube::Ptr tube_ptr);
        void segmentizeCloud(const sensor_msgs::PointCloud2 &cloudIn);
        typedef boost::shared_ptr<TubePerception::CloudProcessing> Ptr;
        bool findDisk(const sensor_msgs::PointCloud2 &clusterCloud,
                      double minRadius, double maxRadius,
                      TubePerception::Cylinder &disk,
                      geometry_msgs::Pose &workPose);

    private:
        bool _convert_cloud_to(std::string target_frame,
                               const sensor_msgs::PointCloud2 &cloud_in,
                               sensor_msgs::PointCloud2 &cloud_out);
        void _segmentize_cloud(pcl::PointCloud<PointT>::Ptr cloud,
                               pcl::PointCloud<PointT>::Ptr hull_points);
        void _estimate_normals(pcl::PointCloud<PointT>::Ptr cloud);
        double _get_radius();
        bool _get_cylinder(pcl::PointCloud<PointT>::Ptr cloud, double r_min,
                           double r_max, pcl::ModelCoefficients &coeff,
                           pcl::PointIndices::Ptr inliers);
        void _collaps_normals(void);
        void _collaps_normals(pcl::PointCloud<PointT>::Ptr cloud_in, double dist, pcl::PointCloud<PointT>::Ptr cloud_out);
        bool _find_line(pcl::PointIndices::Ptr inliers, Cylinder *cyl);
        bool _find_line(pcl::PointCloud<PointT>::Ptr raw_axis_cloud, pcl::PointIndices::Ptr inliers, Cylinder *cyl, double num_of_points);
        void _remove_inliers(pcl::PointCloud<PointT>::Ptr points,
                             pcl::PointIndices::Ptr indices);
        void _remove_inliers(pcl::PointCloud<PointT>::Ptr points,
                             std::vector<int> &indices);
        void _get_line_points(pcl::PointIndices::Ptr inliers,
                              pcl::ModelCoefficients line_coeff,
                              PointT &p1, PointT &p2);
        void _segmentize_axis(void);
        void _compensate_error(void);
        void _cylinder_filter(Cylinder cyl,
                              pcl::PointCloud<PointT>::Ptr cloud_in,
                              pcl::PointIndices::Ptr inliers);
        float _is_in_cylinder( const PointT & pt1, const PointT & pt2,
                               float length_sq, float radius_sq,
                               const PointT & testpt );
        tf::Vector3 _get_perp_vec3(tf::Vector3 v3);
        void _define_pose(void);
        void _generate_work_vectors();
        void _convert_to_pcl(const sensor_msgs::PointCloud2 &rosTubeCloud,
                             pcl::PointCloud<PointT>::Ptr pcl_cloud);
//        bool _convert_cloud_to(std::string target_frame, const sensor_msgs::PointCloud2 &cloud_in, sensor_msgs::PointCloud2 &cloud_out);
        float _r;
        float _r_min, _r_max;
        float _strong_line_thr; //not being used. please remove.
        float _min_points;
        float _weak_line_thr; //not being used. please remove.
        pcl::PointCloud<PointT>::Ptr _raw_axis_points;
        float _z_error;
        int _num_of_points;
        pcl::PointCloud<PointT>::Ptr _tube_cloud;
        pcl::PointCloud<PointT>::Ptr _axis_points;
        TubePerception::Tube::Ptr _tube;
    };

}
#endif // TUBEPERCEPTION_H
