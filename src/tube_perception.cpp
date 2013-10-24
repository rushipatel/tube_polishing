#include "tubePerception.h"
namespace TubePerception
{

Tube::Tube(sensor_msgs::PointCloud2 &tubeCloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(tubeCloud,*cloud);

    tube_cloud_.reset(new pcl::PointCloud<PointT>);
    axis_points_.reset(new pcl::PointCloud<PointT>);

    tube_cloud_->header = cloud->header;
    tube_cloud_->height = cloud->height;
    tube_cloud_->width = cloud->width;
    tube_cloud_->is_dense = cloud->is_dense==1;
    //tube_cloud_->sensor_orientation_ = cloud->sensor_orientation_;
    //tube_cloud_->sensor_origin_ = Eigen::Vector4f (0.0, 0.0, 1.5, 0.0f);

    tube_cloud_->points.resize(cloud->points.size());
    for(unsigned int i=0; i<cloud->points.size(); i++)
    {
        tube_cloud_->points[i].x = cloud->points[i].x;
        tube_cloud_->points[i].y = cloud->points[i].y;
        tube_cloud_->points[i].z = cloud->points[i].z;
        tube_cloud_->points[i].rgb = cloud->points[i].rgb;
    }

    num_of_points_ = tube_cloud_->points.size();
}

void CloudProcessing::processCloud(void)
{
    compensateError();
    estimate_normals_();
    get_radius_();
    collaps_normals_();
    segmentize_axis_();
}

void CloudProcessing::compensateError(void)
{
    for(size_t i=0; i<tube_cloud_->points.size(); i++)
        tube_cloud_->points[i].z -= z_error_;
}

void CloudProcessing::estimate_normals_(void)
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

void CloudProcessing::get_radius_(void)
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

void CloudProcessing::collaps_normals_(void)
{
    raw_axis_points_.reset(new pcl::PointCloud<PointT>);
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

void CloudProcessing::segmentize_axis_(void)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    Cylinder cyl;

    while(find_line_(inliers,&cyl))
    {
        cylinders.push_back(cyl);
        remove_inliers_(raw_axis_points_,inliers);
        inliers->indices.clear();
        cylinder_filter_(cyl, raw_axis_points_, inliers);
        remove_inliers_(raw_axis_points_,inliers);

        inliers->indices.clear();
    }

    axis_points_->header = tube_cloud_->header;
    axis_points_->width = axis_points_->points.size();
    axis_points_->height = 1;
    ROS_INFO("%d Axis points found",axis_points_->points.size());
    ROS_INFO("%d Cylinder found", cylinders.size());
}

bool CloudProcessing::find_line_(pcl::PointIndices::Ptr inliers, Cylinder *cyl)
{
    if(raw_axis_points_->points.size()< (weak_line_thr_*num_of_points_) ) //check if there are enough points to generate model
        return false;
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients coeff;
    //pcl::PointIndices inliers;

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setOptimizeCoefficients(true);
    seg.setProbability(0.99);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.001);
    seg.setInputCloud (raw_axis_points_);
    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers, coeff);


    if( inliers->indices.size()>(strong_line_thr_*num_of_points_) )
    {
        ROS_INFO("Number of Strong Line inliers : %d",inliers->indices.size());
        ROS_INFO("Line coefficients are: [X= %f Y=%f Z=%f] [N_X=%f N_Y=%f N_Z=%f]", coeff.values[0],coeff.values[1],coeff.values[2],coeff.values[3],coeff.values[4],coeff.values[5]);

        get_line_points_(inliers,coeff,cyl);
        cyl->isStrong = true;
        cyl->radius = r_;
        cyl->coefficients = coeff;
        cyl->coefficients.values.push_back(r_);
        return true;
    }
    else
    {
        ROS_INFO("Trying to find weak line");
        seg.setProbability(0.98);
        seg.setMaxIterations (10000);
        seg.setDistanceThreshold (0.002);
        inliers->indices.clear();
        seg.segment (*inliers, coeff);
    }

    if( inliers->indices.size() > (weak_line_thr_*num_of_points_) ) // if confidence in line
    {
        ROS_INFO("Number of Weak Line inliers : %d",inliers->indices.size());
        ROS_INFO("Line coefficients are: [X= %f Y=%f Z=%f] [N_X=%f N_Y=%f N_Z=%f]", coeff.values[0],coeff.values[1],coeff.values[2],coeff.values[3],coeff.values[4],coeff.values[5]);

        get_line_points_(inliers,coeff,cyl);
        cyl->isStrong = false;
        cyl->radius = r_;
        cyl->coefficients = coeff;
        cyl->coefficients.values.push_back(r_);
        return true;
    }

    return false;
}

/*bool CloudProcessing::find_weak_line_(pcl::PointIndices::Ptr inliers, Cylinder *cyl)
{
    if(raw_axis_points_->points.size()<(weak_line_thr_*num_of_points_) )  //check if there are enough points to generate model
        return false;
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients coeff;
    //pcl::PointIndices inliers;

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setOptimizeCoefficients(true);
    seg.setProbability(0.98);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.002);
    seg.setInputCloud (raw_axis_points_);
    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers, coeff);

    ROS_INFO("Number of Weak Line inliers : %d",inliers->indices.size());
    ROS_INFO("Line coefficients are: [X= %f Y=%f Z=%f] [N_X=%f N_Y=%f N_Z=%f]", coeff.values[0],coeff.values[1],coeff.values[2],coeff.values[3],coeff.values[4],coeff.values[5]);

    if( strong_line_thr_ > (inliers->indices.size()/num_of_points_) >= weak_line_thr_ ) // if confidence in line
    {
        get_line_points_(inliers,coeff,cyl);
        cyl->isStrong = false;
        cyl->radius = r_;
        return true;
    }
    else if( inliers->indices.size()>(strong_line_thr_*num_of_points_) )
    {
        get_line_points_(inliers,coeff,cyl);
        cyl->isStrong = true;
        cyl->radius = r_;
        return true;
    }
    return false;
}*/

void CloudProcessing::get_line_points_(pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients line_coeff, Cylinder* cylinder)
{
    pcl::PointCloud<PointT>::Ptr line_points(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr line_inliers(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(raw_axis_points_);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*line_inliers);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    coeff->header = line_coeff.header;
    coeff->values = line_coeff.values;

    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_LINE);
    proj.setInputCloud (line_inliers);
    proj.setModelCoefficients (coeff);
    proj.filter(*line_points);
    PointT p1, p2;
    pcl::getMaxSegment(*line_points,p1,p2);

    cylinder->p1 = p1;
    cylinder->p2 = p2;

    for(size_t i=0; i<line_points->points.size(); i++)
        axis_points_->points.push_back(line_points->points[i]);
}

void CloudProcessing::remove_inliers_(pcl::PointCloud<PointT>::Ptr points, pcl::PointIndices::Ptr indices)
{
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(points);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*points);
}

void CloudProcessing::remove_inliers_(pcl::PointCloud<PointT>::Ptr points,  std::vector<int> &indices)
{
    pcl::ExtractIndices<PointT> extract;
    pcl::PointIndices::Ptr pcl_indices(new pcl::PointIndices);

    pcl_indices->indices.resize(indices.size());
    for(size_t i=0; i<indices.size(); i++)
        pcl_indices->indices[i] = indices[i];

    extract.setInputCloud(points);
    extract.setIndices(pcl_indices);
    extract.setNegative(true);
    extract.filter(*points);
}

float CloudProcessing::is_in_cylinder_( const PointT & pt1, const PointT & pt2, float length_sq, float radius_sq, const PointT & testpt )
{
    float dx, dy, dz;	// vector d  from line segment point 1 to point 2
    float pdx, pdy, pdz;	// vector pd from point 1 to test point
    float dot, dsq;

    dx = pt2.x - pt1.x;	// translate so pt1 is origin.  Make vector from
    dy = pt2.y - pt1.y;     // pt1 to pt2.  Need for this is easily eliminated
    dz = pt2.z - pt1.z;

    pdx = testpt.x - pt1.x;		// vector from pt1 to test point.
    pdy = testpt.y - pt1.y;
    pdz = testpt.z - pt1.z;

    // Dot the d and pd vectors to see if point lies behind the
    // cylinder cap at pt1.x, pt1.y, pt1.z

    dot = pdx * dx + pdy * dy + pdz * dz;

    // If dot is less than zero the point is behind the pt1 cap.
    // If greater than the cylinder axis line segment length squared
    // then the point is outside the other end cap at pt2.

    if( dot < 0.0f || dot > length_sq )
    {
        return( -1.0f );
    }
    else
    {
        // Point lies within the parallel caps, so find
        // distance squared from point to line, using the fact that sin^2 + cos^2 = 1
        // the dot = cos() * |d||pd|, and cross*cross = sin^2 * |d|^2 * |pd|^2
        // Carefull: '*' means mult for scalars and dotproduct for vectors
        // In short, where dist is pt distance to cyl axis:
        // dist = sin( pd to d ) * |pd|
        // distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
        // dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
        // dsq = pd * pd - dot * dot / lengthsq
        //  where lengthsq is d*d or |d|^2 that is passed into this function

        // distance squared to the cylinder axis:

        dsq = (pdx*pdx + pdy*pdy + pdz*pdz) - dot*dot/length_sq;

        if( dsq > radius_sq )
        {
            return( -1.0f );
        }
        else
        {
            return( dsq );		// return distance squared to axis
        }
    }
}

void CloudProcessing::cylinder_filter_(Cylinder cyl, pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointIndices::Ptr inliers)
{
    PointT p1, p2;
    p1 = cyl.p1;
    p2 = cyl.p2;
    int pts_cnt = 0;
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;


    float l_sq = (dx*dx) + (dy*dy) + (dz*dz);
    float r_sq = r_*r_;

    for(size_t i=0; i<cloud_in->points.size(); i++)
    {
        if(is_in_cylinder_(p1, p2, l_sq, r_sq, cloud_in->points[i])>0)
        {
            inliers->indices.push_back(i);
            pts_cnt++;
        }
    }
    ROS_INFO("%d Points found in cylinder filter",pts_cnt);
}

void CloudProcessing::setZerror(float error)
{
    z_error_ = error;
}

void CloudProcessing::displayCloud(int sec)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (tube_cloud_,"tube_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tube_cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->spin();
}
void CloudProcessing::displayAxisPoints(int sec)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (axis_points_,"axis");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "axis");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->spin();
}

void CloudProcessing::displayCylinders(int sec)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::ModelCoefficients coeffs;
    coeffs.values.resize(7);
    for (size_t i=0; i<cylinders.size(); i++)
    {
        coeffs.header = cylinders[i].coefficients.header;
        coeffs.values[0] = cylinders[i].p1.x;
        coeffs.values[1] = cylinders[i].p1.y;
        coeffs.values[2] = cylinders[i].p1.z;
        coeffs.values[3] = cylinders[i].p2.x-cylinders[i].p1.x;
        coeffs.values[4] = cylinders[i].p2.y-cylinders[i].p1.y;
        coeffs.values[5] = cylinders[i].p2.z-cylinders[i].p1.z;
        coeffs.values[6] = cylinders[i].radius;
        std::strstream ss;
        ss<<"Cylinder_"<<i;
        viewer->addCylinder(coeffs,ss.str());
    }
    viewer->addPointCloud<PointT> (axis_points_,"LinePoints");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "LinePoints");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->spin();
}

void CloudProcessing::displayLines(int sec)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::ModelCoefficients coeffs;
    coeffs.header = cylinders[0].coefficients.header;
    coeffs.values.resize(6);
    for (size_t i=0; i<cylinders.size(); i++)
    {
        /*coeffs.values[0] = cylinders[i].p1.x;
        coeffs.values[1] = cylinders[i].p1.y;
        coeffs.values[2] = cylinders[i].p1.z;
        coeffs.values[3] = cylinders[i].p2.x-cylinders[i].p1.x;
        coeffs.values[4] = cylinders[i].p2.y-cylinders[i].p1.y;
        coeffs.values[5] = cylinders[i].p2.z-cylinders[i].p1.z;*/

        std::strstream ss;
        ss<<"Line_"<<i;
        viewer->addLine(cylinders[i].p1,cylinders[i].p2,ss.str());
    }
    viewer->addPointCloud<PointT> (axis_points_,"LinePoints");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "LinePoints");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->spin();
}

}
