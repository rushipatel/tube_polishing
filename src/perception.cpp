#include "perception.h"

struct line
{
    PointT p1;
    PointT p2;
};

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
//-----------------------------------------------------------------------------
// Name: CylTest_CapsFirst
// Orig: Greg James - gjames@NVIDIA.com
// Lisc: Free code - no warranty & no money back.  Use it all you want
// Desc:
//    This function tests if the 3D point 'testpt' lies within an arbitrarily
// oriented cylinder.  The cylinder is defined by an axis from 'pt1' to 'pt2',
// the axis having a length squared of 'lengthsq' (pre-compute for each cylinder
// to avoid repeated work!), and radius squared of 'radius_sq'.
//    The function tests against the end caps first, which is cheap -> only
// a single dot product to test against the parallel cylinder caps.  If the
// point is within these, more work is done to find the distance of the point
// from the cylinder axis.
//    Fancy Math (TM) makes the whole test possible with only two dot-products
// a subtract, and two multiplies.  For clarity, the 2nd mult is kept as a
// divide.  It might be faster to change this to a mult by also passing in
// 1/lengthsq and using that instead.
//    Elminiate the first 3 subtracts by specifying the cylinder as a base
// point on one end cap and a vector to the other end cap (pass in {dx,dy,dz}
// instead of 'pt2' ).
//
//    The dot product is constant along a plane perpendicular to a vector.
//    The magnitude of the cross product divided by one vector length is
// constant along a cylinder surface defined by the other vector as axis.
//
// Return:  -1.0 if point is outside the cylinder
// Return:  distance squared from cylinder axis if point is inside.
//
//-----------------------------------------------------------------------------
//   http://www.flipcode.com/archives/Fast_Point-In-Cylinder_Test.shtml

/*struct Vec3
{
    float x;
    float y;
    float z;
};*/

float is_in_cylinder( const PointT & pt1, const PointT & pt2, float length_sq, float radius_sq, const PointT & testpt )
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

void points_in_cylinder(line l, pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointIndices::Ptr inliers)
{
    PointT p1, p2;
    p1 = l.p1;
    p2 = l.p2;
    int pts_cnt = 0;
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;

    float r = 0.01;

    float l_sq = (dx*dx) + (dy*dy) + (dz*dz);
    float r_sq = r*r;

    for(size_t i=0; i<cloud_in->points.size(); i++)
    {
        if(is_in_cylinder(p1, p2, l_sq, r_sq, cloud_in->points[i])>0)
        {
            inliers->indices.push_back(i);
            pts_cnt++;
        }
    }
    ROS_INFO("%d Points found in cylinder filter",pts_cnt);
}

void remove_inliers(pcl::PointCloud<PointT>::Ptr points, pcl::PointIndices::Ptr indices)
{
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(points);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*points);
}

void remove_inliers(pcl::PointCloud<PointT>::Ptr points,  std::vector<int> &indices)
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

bool ransac_line(pcl::PointCloud<PointT>::Ptr axis_points, pcl::ModelCoefficients::Ptr coefficient, pcl::PointIndices::Ptr inliers)
{
    if(axis_points->points.size()<20) //check if there are enough points to generate model
    {
        ROS_INFO("No more points to fit line model");
        return false;
    }
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
    seg.setDistanceThreshold (0.0005);  //This will determine smallest cylinder
    seg.setInputCloud (axis_points);
    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers, coeff);
    coefficient->header = coeff.header;
    coefficient->values = coeff.values;

    ROS_INFO("Number of Line inliers : %d",inliers->indices.size());
    ROS_INFO("Line coefficients are: [X= %f Y=%f Z=%f] [N_X=%f N_Y=%f N_Z=%f]", coeff.values[0],coeff.values[1],coeff.values[2],coeff.values[3],coeff.values[4],coeff.values[5]);

    if(inliers->indices.size()>20) // if confidence in line
        return true;
    else
        return false;
}

void get_line_points(pcl::PointCloud<PointT>::Ptr axis_points, pcl::PointIndices::Ptr inliers, pcl::PointCloud<PointT>::Ptr axis_line_points, pcl::ModelCoefficients::Ptr line_coeff, line &ln)
{
    pcl::PointCloud<PointT>::Ptr line_points(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr line_inliers(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(axis_points);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*line_inliers);

    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_LINE);
    proj.setInputCloud (line_inliers);
    proj.setModelCoefficients (line_coeff);
    proj.filter(*line_points);
    PointT p1, p2;
    pcl::getMaxSegment(*line_points,p1,p2);

    ln.p1 = p1;
    ln.p2 = p2;

    for(size_t i=0; i<line_points->points.size(); i++)
        axis_line_points->points.push_back(line_points->points[i]);
}


int segmentize_axis(pcl::PointCloud<PointT>::Ptr axis_points, pcl::PointCloud<PointT>::Ptr axis_line_points)
{
    pcl::ModelCoefficients::Ptr line_coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    std::vector<line> ln_vec;
    line ln;
    int idx;

    while(ransac_line(axis_points,line_coeff,inliers))
    {
        get_line_points(axis_points,inliers,axis_line_points,line_coeff,ln);
        remove_inliers(axis_points,inliers);
        ln_vec.push_back(ln);
        inliers->indices.clear();
        points_in_cylinder(ln, axis_points, inliers);
        remove_inliers(axis_points,inliers);

        inliers->indices.clear();
    }

    axis_line_points->header = axis_points->header;
    axis_line_points->width = axis_line_points->points.size();
    axis_line_points->height = 1;
    ROS_INFO("%d axis points found",axis_line_points->points.size());
    ROS_INFO("%d Lines found", ln_vec.size());
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
    //display_normals(cloud);
    r = get_radius(cloud);

    pcl::PointCloud<PointT>::Ptr axis_points(new pcl::PointCloud<PointT>);
    get_axis_points(cloud, r, axis_points);

    ROS_INFO("Width: %d   Height: %d   Size: %d",axis_points->width, axis_points->height, axis_points->points.size());

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    //viewer = pointsVis(axis_points);
    //display_cloud(viewer);
    //viewer->addPointCloud<PointT>(cloud,"tube_axis");
    //display_cloud(viewer);
    pcl::PointCloud<PointT>::Ptr axis_line_points(new pcl::PointCloud<PointT>);
    segmentize_axis(axis_points,axis_line_points);
    viewer = pointsVis(axis_line_points);
    display_cloud(viewer);
    ROS_INFO("Number of axis_line_points is: %d", axis_line_points->points.size());
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
