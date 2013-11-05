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

geometry_msgs::Pose Cylinder::getPose(void)
{
    return pose;
}

tf::Transform Cylinder::getTransform(void)
{
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    tf::Quaternion q;
    q.setX(pose.orientation.x);
    q.setY(pose.orientation.y);
    q.setZ(pose.orientation.z);
    q.setW(pose.orientation.w);
    tf.setRotation(q);
    return tf;
}

void CloudProcessing::processCloud_(void)
{
    compensate_error_();
    estimate_normals_();
    get_radius_();
    collaps_normals_();
    segmentize_axis_();
    define_pose_();
    //get_line_graph_();
    //get_curves_();
    //print_line_graph_();
}

bool CloudProcessing::writeAxisPointsOnFile(std::string fileName)
{
    if(pcl::io::savePCDFileASCII(fileName,*axis_points_))
        return true;
    return false;
}

tf::Vector3 CloudProcessing::get_perp_vec3_(tf::Vector3 v3)
{
    tf::Vector3 perp_vec, unit_vec;
    unit_vec.setZero();
    int a = v3.furthestAxis(); //0,1,2=x,y,z
    if(a==1)
        unit_vec.setY(1);
    else if(a==2)
        unit_vec.setZ(1);
    else
        unit_vec.setX(1);
    perp_vec = v3.cross(unit_vec);
    perp_vec.normalize();
    return perp_vec;
}

//cylindrical axis is X
void CloudProcessing::define_pose_(void)
{
    tf::Vector3 v1, v2, v3;
    tf::Matrix3x3 mat;
    tf::Quaternion q;
    for(size_t i=0; i<cylinders.size(); i++)
    {
        cylinders[i].pose.position.x = (cylinders[i].p1.x); //+ cylinders[i].p2.x) / 2;
        cylinders[i].pose.position.y = (cylinders[i].p1.y); //+ cylinders[i].p2.y) / 2;
        cylinders[i].pose.position.z = (cylinders[i].p1.z); //+ cylinders[i].p2.z) / 2;

        v1.setX(cylinders[i].p2.x - cylinders[i].p1.x);
        v1.setY(cylinders[i].p2.y - cylinders[i].p1.y);
        v1.setZ(cylinders[i].p2.z - cylinders[i].p1.z);
        cylinders[i].axisVector = v1;
        v2 = get_perp_vec3_(v1);
        v1.normalize();
        v3 = v1.cross(v2);
        v3.normalize();

        mat.setValue(v1.x(),v1.y(),v1.z(), v2.x(), v2.y(),v2.z(),v3.x(),v3.y(),v3.z());
        mat.getRotation(q);
        cylinders[i].pose.orientation.x = q.x();
        cylinders[i].pose.orientation.y = q.y();
        cylinders[i].pose.orientation.z = q.z();
        cylinders[i].pose.orientation.w = q.w();
    }
}

void CloudProcessing::get_line_graph_(void)
{
    float dist;
    for(size_t i=0; i<cylinders.size(); i++)
    {
        for(size_t j=0; j<cylinders.size(); j++)
        {
            if(i!=j)
            {
                dist = pcl::euclideanDistance(cylinders[i].p1, cylinders[j].p1);
                if( dist < (r_) )
                {
                    std::cout<<"adding "<<j<<" in "<<i<<std::endl;
                    add_neighbour_(i, j);
                }

                dist = pcl::euclideanDistance(cylinders[i].p2, cylinders[j].p1);
                if( dist < (r_) )
                {
                    std::cout<<"adding "<<j<<" in "<<i<<std::endl;
                    add_neighbour_(i, j);
                }

                dist = pcl::euclideanDistance(cylinders[i].p1, cylinders[j].p2);
                if( dist < (r_) )
                {
                    std::cout<<"adding "<<j<<" in "<<i<<std::endl;
                    add_neighbour_(i, j);
                }

                dist = pcl::euclideanDistance(cylinders[i].p2, cylinders[j].p2);
                if( dist < (r_) )
                {
                    std::cout<<"adding "<<j<<" in "<<i<<std::endl;
                    add_neighbour_(i, j);
                }
            }
        }
    }
}

void CloudProcessing::add_neighbour_(int cyl_ind, int neighbour_ind)
{
    for(size_t i=0; i<cylinders[cyl_ind].neighbourCylinders.size(); i++)
    {
        if(cylinders[cyl_ind].neighbourCylinders[i]==neighbour_ind)
            return;
    }
        cylinders[cyl_ind].neighbourCylinders.push_back(neighbour_ind);
}

void CloudProcessing::print_line_graph_(void)
{
    for(size_t i=0; i<cylinders.size(); i++)
    {
        std::cout<<std::endl<<"Cylinder "<<i<<" :";
        for(size_t j=0; j<cylinders[i].neighbourCylinders.size(); j++)
            std::cout<<std::endl<<"\t\t"<<cylinders[i].neighbourCylinders[j];
    }
}

/*void CloudProcessing::group_cylinders_(void)
{
    std::vector<int> cyl_ind_stack;

    int curve_ind = 0;
    int cyl_ind;
    for(size_t i=0; i<cylinders.size(); i++)
    {
        cyl_ind_stack.push_back(i);
        while(!cyl_ind_stack.empty())
        {
            cyl_ind = cyl_ind_stack.pop_back();
        }
    }
}*/

void CloudProcessing::compensate_error_(void)
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
    seg.setMaxIterations (20000);
    seg.setDistanceThreshold (0.01);
    seg.setRadiusLimits (0.001, 0.2);
    seg.setProbability(0.99);
    seg.setInputCloud (tube_cloud_);
    seg.setInputNormals (tube_cloud_);
    seg.setDistanceFromOrigin(0.05);
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
    PointT p1, p2;

    while(find_line_(inliers,&cyl))
    {
        cylinders.push_back(cyl);
        remove_inliers_(raw_axis_points_,inliers);

        inliers->indices.clear();

        cylinder_filter_(cyl, raw_axis_points_, inliers);
        get_line_points_(inliers,cyl.coefficients, p1, p2); // no use for p1 & p2
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
    if(raw_axis_points_->points.size()< (min_points_*num_of_points_) ) //check if there are enough points to generate model
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
    seg.setMaxIterations (20000);
    seg.setDistanceThreshold (0.001);
    seg.setInputCloud (raw_axis_points_);
    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers, coeff);

    PointT p1,p2;

    if( inliers->indices.size()>(min_points_*num_of_points_) )
    {
        ROS_INFO("Number of Strong Line inliers : %d",inliers->indices.size());
        ROS_INFO("Line coefficients are: [X= %f Y=%f Z=%f] [N_X=%f N_Y=%f N_Z=%f]", coeff.values[0],coeff.values[1],coeff.values[2],coeff.values[3],coeff.values[4],coeff.values[5]);

        get_line_points_(inliers,coeff, p1, p2);
        cyl->p1 = p1;
        cyl->p2 = p2;
        cyl->isStrong = true;
        cyl->radius = r_;
        cyl->coefficients.header = coeff.header;
        cyl->coefficients.values.resize(7);
        cyl->coefficients.values[0] = p1.x;
        cyl->coefficients.values[1] = p1.y;
        cyl->coefficients.values[2] = p1.z;
        cyl->coefficients.values[3] = p2.x - p1.x;
        cyl->coefficients.values[4] = p2.y - p1.y;
        cyl->coefficients.values[5] = p2.z - p1.z;
        cyl->coefficients.values[6] = r_;
        return true;
    }
    else
    {
        ROS_INFO("Trying to find weak line...");
        seg.setProbability(0.97);
        seg.setMaxIterations (10000);
        seg.setDistanceThreshold (0.002);
        inliers->indices.clear();
        seg.segment (*inliers, coeff);
    }

    if( inliers->indices.size() > (min_points_*num_of_points_) ) // if confidence in line
    {
        ROS_INFO("Number of Weak Line inliers : %d",inliers->indices.size());
        ROS_INFO("Line coefficients are: [X= %f Y=%f Z=%f] [N_X=%f N_Y=%f N_Z=%f]", coeff.values[0],coeff.values[1],coeff.values[2],coeff.values[3],coeff.values[4],coeff.values[5]);

        get_line_points_(inliers,coeff, p1, p2);
        cyl->p1 = p1;
        cyl->p2 = p2;
        cyl->isStrong = false;
        cyl->radius = r_;
        cyl->coefficients.values.resize(7);
        cyl->coefficients.values[0] = p1.x;
        cyl->coefficients.values[1] = p1.y;
        cyl->coefficients.values[2] = p1.z;
        cyl->coefficients.values[3] = p2.x - p1.x;
        cyl->coefficients.values[4] = p2.y - p1.y;
        cyl->coefficients.values[5] = p2.z - p1.z;
        cyl->coefficients.values[6] = r_;
        return true;
    }

    return false;
}

void CloudProcessing::get_line_points_(pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients line_coeff, PointT &p1, PointT &p2)
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
    coeff->values.resize(6);
    for(int i=0; i<6; i++)
        coeff->values[i] = line_coeff.values[i];

    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_LINE);
    proj.setInputCloud (line_inliers);
    proj.setModelCoefficients (coeff);
    proj.filter(*line_points);
    //PointT p1, p2;
    pcl::getMaxSegment(*line_points,p1,p2);

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
    //float sum;
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
        /*ss<<"_weld";
        sum = coeffs.values[3]+coeffs.values[4]+coeffs.values[5];
        sum = 0.005/sum;
        if(!cylinders[i].pointsOfInterest.empty())
        {
            coeffs.values[0] = cylinders[i].pointsOfInterest[0].x;
            coeffs.values[1] = cylinders[i].pointsOfInterest[0].y;
            coeffs.values[2] = cylinders[i].pointsOfInterest[0].z;
            coeffs.values[3] *= sum;
            coeffs.values[4] *= sum;
            coeffs.values[5] *= sum;
            coeffs.values[6] = cylinders[i].radius + 0.0015;
            viewer->addCylinder(coeffs,ss.str());
        }*/
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

}//TubePerception
