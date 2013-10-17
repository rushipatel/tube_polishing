#include "perception.h"


//hase to contain normals
typedef pcl::PointXYZRGBNormal PointT;

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (pcl::PointCloud<PointT>::ConstPtr cloud)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PointT> (cloud,"tube_cloud_normals");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "tube_cloud_normals");
  viewer->addPointCloudNormals<PointT, PointT> (cloud, cloud, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> pointsVis (pcl::PointCloud<PointT>::ConstPtr cloud)
{
  // --------------------------------------------------------
  // ----------Open 3D viewer and add point cloud------------
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PointT> (cloud,"tube_cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tube_cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> init_pointsVis_addpoints(pcl::PointCloud<PointT>::ConstPtr cloud)
{
  // --------------------------------------------------------
  // ----------Open 3D viewer and add point cloud------------
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PointT> (cloud,"tube_cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tube_cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void points_vis_addpoints(pcl::PointCloud<PointT>::ConstPtr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, const std::string& cloud_name)
{
    viewer->addPointCloud<PointT> (cloud,cloud_name);
}

void estimate_normals(pcl::PointCloud<PointT>::Ptr cloud_in)
{
    pcl::NormalEstimation<PointT, PointT> ne;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_in);
    ne.setRadiusSearch(0.01);
    //ROS_INFO("Setting view point to sensor_origin_= ")cloud_in->sensor_origin_;
    ne.setViewPoint(0.0,0.0,1.0);
    ne.compute(*cloud_in);
}

double get_radius(pcl::PointCloud<PointT>::Ptr cloud_in)
{
    pcl::SACSegmentationFromNormals<PointT, PointT> seg;
    pcl::ModelCoefficients coeff;
    pcl::PointIndices inliers;

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.01);
    seg.setRadiusLimits (0.001, 0.2);
    seg.setInputCloud (cloud_in);
    seg.setInputNormals (cloud_in);
    seg.setDistanceFromOrigin(0.01);

    // Obtain the cylinder inliers and coefficients
    seg.segment (inliers, coeff);

    ROS_INFO("Trying with cylinder Radius: %f", coeff.values[6]);
    ROS_INFO("Number of Inliers for RANSAC cylinder: %d",inliers.indices.size());

    return coeff.values[6];

}

void copy_point_cloud(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointIndices&  indices, pcl::PointCloud<PointT>::Ptr cloud_out)
{
    if(indices.indices.size() > cloud_in->points.size())
    {
        ROS_ERROR("copy_point_cloud: number of indices are more than cloud_in");
        return;
    }
    else
    {
        cloud_out->header = cloud_in->header;
        cloud_out->height = 1;
        cloud_out->width = indices.indices.size();
        cloud_out->points.resize(indices.indices.size());
        for(size_t i=0; i<indices.indices.size(); i++)
        {
            cloud_out->points[i].x = cloud_in->points[indices.indices[i]].x;
            cloud_out->points[i].y = cloud_in->points[indices.indices[i]].y;
            cloud_out->points[i].z = cloud_in->points[indices.indices[i]].z;

            //std::cout<<i<<" ";
        }
    }
}

void get_largest_cluster(pcl::PointCloud<PointT>::Ptr points_in, pcl::PointCloud<PointT>::Ptr points_out)
{
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(points_in);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.01); // 1cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (points_in->points.size());
    ec.setSearchMethod (tree);
    ec.setInputCloud (points_in);
    ec.extract (clusters);
    if(!clusters.empty())
    {
        if(clusters.size()==1)
        {
            ROS_INFO("One cluster found from raw_axis_points");
            copy_point_cloud(points_in, clusters[0], points_out);
        }
        else
        {
            int max = 0;
            int cluster_no = 0;
            for(size_t i=0; i<clusters.size(); i++)
            {
                if(clusters[i].indices.size()>max)
                {
                    cluster_no = i;
                    max = clusters[i].indices.size();
                }
            }
            ROS_WARN("Number(%d) of clusters is greater than one. Consider optimizing clustering parameters",clusters.size());
            ROS_INFO("Using cluster %d with highest number of points %d",cluster_no+1,max);
            copy_point_cloud(points_in, clusters[cluster_no], points_out);
        }
    }
    else
        ROS_ERROR("Cluster is empty, cannot get axis points");
}

void get_axis_points(pcl::PointCloud<PointT>::Ptr cloud_in, double r, pcl::PointCloud<PointT>::Ptr axis_points)
{
    pcl::PointCloud<PointT>::Ptr raw_axis_points(new pcl::PointCloud<PointT>);
    raw_axis_points->header = cloud_in->header;
    raw_axis_points->points.resize(cloud_in->points.size());

    for(unsigned int i=0; i<cloud_in->points.size(); i++)
    {
        raw_axis_points->points[i].x = cloud_in->points[i].x - cloud_in->points[i].normal_x * r;
        raw_axis_points->points[i].y = cloud_in->points[i].y - cloud_in->points[i].normal_y * r;
        raw_axis_points->points[i].z = cloud_in->points[i].z - cloud_in->points[i].normal_z * r;
    }

    raw_axis_points->width = raw_axis_points->points.size();
    raw_axis_points->height = 1;

    get_largest_cluster(raw_axis_points, axis_points);

}

void ransac_line(pcl::PointCloud<PointT>::Ptr axis_points, pcl::ModelCoefficients::Ptr coefficient)
{
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients coeff;
    pcl::PointIndices inliers;

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.003);
    seg.setInputCloud (axis_points);

    // Obtain the cylinder inliers and coefficients
    seg.segment (inliers, coeff);

    coefficient->header = coeff.header;
    coefficient->values = coeff.values;

    ROS_INFO("Number of Line inliers : %d",inliers.indices.size());
    ROS_INFO("Line coefficients are: [X= %f Y=%f Z=%f] [N_X=%f N_Y=%f N_Z=%f]", coeff.values[0],coeff.values[1],coeff.values[2],coeff.values[3],coeff.values[4],coeff.values[5]);
}

void process_tube_cloud(sensor_msgs::PointCloud2 &object_cloud)
{
    double r = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(object_cloud,*cloud2);

    /*std::cout<<"******************"<<std::endl;
    std::cout<<cloud2->sensor_origin_;*/

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    cloud->header = cloud2->header;
    cloud->height = cloud2->height;
    cloud->width = cloud2->width;
    cloud->is_dense = cloud2->is_dense==1;
    cloud->sensor_orientation_ = cloud2->sensor_orientation_;
    cloud->sensor_origin_ = cloud2->sensor_origin_;
    cloud->points.resize(cloud2->points.size());

    for(unsigned int i=0; i<cloud2->points.size(); i++)
    {
        cloud->points[i].x = cloud2->points[i].x;
        cloud->points[i].y = cloud2->points[i].y;
        cloud->points[i].z = cloud2->points[i].z;
        cloud->points[i].rgb = cloud2->points[i].rgb;
    }
    estimate_normals(cloud);
    display_normals(cloud);
    r = get_radius(cloud);

    pcl::PointCloud<PointT>::Ptr axis_points(new pcl::PointCloud<PointT>);
    get_axis_points(cloud, r, axis_points);

    ROS_INFO("Width: %d   Height: %d   Size: %d",axis_points->width, axis_points->height, axis_points->points.size());

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = pointsVis(axis_points);
    display_cloud(viewer);
    viewer->addPointCloud<PointT>(cloud,"tube_axis");
    //display_cloud(viewer);

    //do further analysis
    pcl::ModelCoefficients::Ptr line_coeff(new pcl::ModelCoefficients);
    ransac_line(axis_points, line_coeff);
}

void display_cloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100, true);
        boost::this_thread::sleep(boost::posix_time::microseconds (100000));
    }
}

void display_normals(pcl::PointCloud<PointT>::Ptr cloud_with_normals)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = normalsVis(cloud_with_normals);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep(boost::posix_time::microseconds (100000));
    }
}
