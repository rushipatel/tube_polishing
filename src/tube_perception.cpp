#include "tubePerception.h"

tubePerception::tubePerception(sensor_msgs::PointCloud2 &tubeCloud)
{
    //ros_tube_cloud = tubeCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(tubeCloud,*cloud);

    tube_cloud_->header = cloud->header;
    tube_cloud_->height = cloud->height;
    tube_cloud_->width = cloud->width;
    tube_cloud_->is_dense = cloud->is_dense==1;
    //tube_cloud_->sensor_orientation_ = cloud->sensor_orientation_;
    tube_cloud_->sensor_origin_ = Eigen::Vector4f (0.0, 0.0, 1.5, 0.0f);

            tube_cloud_->points.resize(cloud->points.size());
    for(unsigned int i=0; i<cloud->points.size(); i++)
    {
        tube_cloud_->points[i].x = cloud->points[i].x;
        tube_cloud_->points[i].y = cloud->points[i].y;
        tube_cloud_->points[i].z = cloud->points[i].z;
        tube_cloud_->points[i].rgb = cloud->points[i].rgb;
    }
}

void tubePerception::processCloud(void)
{
    estimate_normals_();
}

void tubePerception::estimate_normals_(void)
{
    pcl::NormalEstimation<PointT, PointT> ne;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(tube_cloud_);

    ne.setRadiusSearch(0.01);
    ROS_INFO("Using view point x=0.0 y=0.0 z=1.5");
    ne.setViewPoint(0.0,0.0,1.5);
    ne.compute(*tube_cloud_);
}

void tubePerception::get_radius_(void)
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
    seg.setInputCloud (tube_cloud_);
    seg.setInputNormals (tube_cloud_);
    seg.setDistanceFromOrigin(0.01);
    // Obtain the cylinder inliers and coefficients
    seg.segment (inliers, coeff);
    ROS_INFO("Foud cylinder radius using cylinder RANSAC: %f", coeff.values[6]);
    r_ = coeff.values[6]; //r_.push_back(coeff.values[6]);
}

void tubePerception::collaps_normals_(void)
{
    pcl::PointCloud<PointT>::Ptr raw_axis_points_(new pcl::PointCloud<PointT>);
    raw_axis_points_->header = tube_cloud_->header;
    raw_axis_points_->points.resize(tube_cloud_->points.size());

    for(unsigned int i=0; i<tube_cloud_->points.size(); i++)
    {
        raw_axis_points_->points[i].x = tube_cloud_->points[i].x - tube_cloud_->points[i].normal_x * r_;
        raw_axis_points_->points[i].y = tube_cloud_->points[i].y - tube_cloud_->points[i].normal_y * r_;
        raw_axis_points_->points[i].z = tube_cloud_->points[i].z - tube_cloud_->points[i].normal_z * r_;
    }

    raw_axis_points_->width = raw_axis_points_->points.size();
    raw_axis_points_->height = 1;

    //get_largest_cluster(raw_axis_points, axis_points);
}

void tubePerception::segmentize_tube_(void)
{
    pcl::ModelCoefficients::Ptr line_coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    line ln;

    while(find_strong_line(line_coeff,inliers))
    {
        get_line_points(inliers,line_coeff,ln);
        remove_inliers(raw_axis_points_,inliers);
        lines.push_back(ln);
        inliers->indices.clear();
        cylinder_filter(ln, raw_axis_points_, inliers);
        remove_inliers(raw_axis_points_,inliers);

        inliers->indices.clear();
    }

    while(find_weak_line(line_coeff,inliers))
    {

        remove_inliers(raw_axis_points_,inliers);
        lines.push_back(ln);
        inliers->indices.clear();
        cylinder_filter(ln, raw_axis_points_, inliers);
        remove_inliers(raw_axis_points_,inliers);

        inliers->indices.clear();
    }

    axis_line_points->header = axis_points->header;
    axis_line_points->width = axis_line_points->points.size();
    axis_line_points->height = 1;
    ROS_INFO("%d axis points found",axis_line_points->points.size());
    ROS_INFO("%d Lines found", ln_vec.size());
}

bool tubePerception::find_strong_line_(pcl::ModelCoefficients::Ptr coefficient, pcl::PointIndices::Ptr inliers, tubePerception::line &ln)
{
    if(raw_axis_points_->points.size()<strong_line_points_) //check if there are enough points to generate model
        return false;
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients coeff;
    //pcl::PointIndices inliers;

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setOptimizeCoefficients(true);
    seg.setProbability(0.985);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.001);  //This will determine smallest cylinder
    seg.setInputCloud (axis_points);
    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers, coeff);
    coefficient->header = coeff.header;
    coefficient->values = coeff.values;

    ROS_INFO("Number of Line inliers : %d",inliers->indices.size());
    ROS_INFO("Line coefficients are: [X= %f Y=%f Z=%f] [N_X=%f N_Y=%f N_Z=%f]", coeff.values[0],coeff.values[1],coeff.values[2],coeff.values[3],coeff.values[4],coeff.values[5]);

    if(inliers->indices.size()>strong_line_points_) // if confidence in line
    {
        get_line_points(inliers,line_coeff,ln);
        ln.isStrong = true;
        return true;
    }
    return false;
}
