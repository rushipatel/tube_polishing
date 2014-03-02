#include "tubePerception.h"
namespace TubePerception
{

Tube::Tube(sensor_msgs::PointCloud2 &rosTubeCloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(rosTubeCloud,*cloud);

    tubeCloud.reset(new pcl::PointCloud<PointT>);
    axisPoints.reset(new pcl::PointCloud<PointT>);

    tubeCloud->header = cloud->header;
    tubeCloud->height = cloud->height;
    tubeCloud->width = cloud->width;
    tubeCloud->is_dense = cloud->is_dense==1;
    //tubeCloud->sensor_orientation_ = cloud->sensor_orientation_;
    //tubeCloud->sensor_origin_ = Eigen::Vector4f (0.0, 0.0, 1.5, 0.0f);

    tubeCloud->points.resize(cloud->points.size());
    for(unsigned int i=0; i<cloud->points.size(); i++)
    {
        tubeCloud->points[i].x = cloud->points[i].x;
        tubeCloud->points[i].y = cloud->points[i].y;
        tubeCloud->points[i].z = cloud->points[i].z;
        tubeCloud->points[i].rgb = cloud->points[i].rgb;
    }
}

void Tube::setPose(geometry_msgs::Pose &pose)
{
    _pose = pose;
}

geometry_msgs::Pose Tube::getPose()
{
    return _pose;
}

tf::Transform Tube::getTransform(void)
{
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(_pose.position.x, _pose.position.y, _pose.position.z));
    tf::Quaternion q;
    q.setX(_pose.orientation.x);
    q.setY(_pose.orientation.y);
    q.setZ(_pose.orientation.z);
    q.setW(_pose.orientation.w);
    tf.setRotation(q);
    return tf;
}

geometry_msgs::Pose Tube::getCylinderGlobalPose(unsigned int cylIdx)
{
    if(cylinders.empty())
        ROS_ERROR("cylinders object is empty");
    tf::Transform c, t = this->getTransform();
    TubePerception::Cylinder cyl = this->cylinders.at(cylIdx);
    c = cyl.getLocalTransform();
    c = t*c;
    return tf2pose(c);
}

void Tube::setPoseAsActualPose()
{
    _actual_pose = _pose;
}
geometry_msgs::Pose Tube::getActualPose()
{
    return _actual_pose;
}

float isInCylinder( const PointT & pt1, const PointT & pt2, float length_sq, float radius_sq, const PointT & testpt )
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

//point with normal
unsigned int Tube::whichCylinder(PointT point)
{
    //tf::Vector3 normal;
    tf::Vector3 p;
    /*normal.setValue(point.normal_x, point.normal_y, point.normal_z);
    normal.normalize();
    normal *= -0.005; // travel in reverse by 5 mm so it can lie inside some cylinder
    point.x = point.x + normal.x();
    point.y = point.y + normal.y();
    point.z = point.z + normal.z();*/
    for(size_t i=0; i<cylinders.size(); i++)
    {
        PointT p1 = cylinders[i].p1;
        PointT p2 = cylinders[i].p2;

        float l_sqr = ((p1.x - p2.x) * (p1.x - p2.x)) +
                      ((p1.y - p2.y) * (p1.y - p2.y)) +
                      ((p1.z - p2.z) * (p1.z - p2.z)) ;
        //inflate cylinder so that point can lie inside in case of outlier.
        //in other words, 3mm tollerance.
        double r_sqr = (cylinders[i].radius+0.003)*(cylinders[i].radius+0.003);
        if(isInCylinder(p1,p2,l_sqr,r_sqr,point)>0)
            return i;
    }
    return cylinders.size();
}

//pre requisites: right and/or left grasp
arm_navigation_msgs::AttachedCollisionObject
Tube::getAttachedObjForRightGrasp(geometry_msgs::Pose &right_grasp_pose)
{
    arm_navigation_msgs::AttachedCollisionObject obj;
    _get_attached_collision_object(obj, right_grasp_pose, "r_wrist_roll_link", true, false);
    return obj;
}

arm_navigation_msgs::AttachedCollisionObject
Tube::getAttachedObjForLeftGrasp(geometry_msgs::Pose &left_grasp_pose)
{
    arm_navigation_msgs::AttachedCollisionObject obj;
    _get_attached_collision_object(obj, left_grasp_pose, "l_wrist_roll_link", false, true);
    return obj;
}

arm_navigation_msgs::AttachedCollisionObject
Tube::getAttachedObjForBothGrasps(geometry_msgs::Pose &right_grasp_pose)
{
    arm_navigation_msgs::AttachedCollisionObject obj;
    _get_attached_collision_object(obj, right_grasp_pose, "r_wrist_roll_link", true, true);
    return obj;
}

//from grasp to tube
void Tube::_get_attached_collision_object(arm_navigation_msgs::AttachedCollisionObject &obj, geometry_msgs::Pose &grasp_pose,
                                     std::string link_name,
                                     bool right_side, bool left_side)
{
    obj.link_name = link_name;
    obj.object.header.frame_id = link_name;

    if(link_name.compare("r_wrist_roll_link"))
        right_side = true;
    if(link_name.compare("l_wrist_roll_link"))
        left_side = true;

    if(right_side)
        obj.touch_links.push_back("r_end_effector");
    if(left_side)
        obj.touch_links.push_back("l_end_effector");

    obj.object.id = "tube";
    obj.object.header.stamp = ros::Time::now();
    obj.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    arm_navigation_msgs::Shape cyl_shape;
    cyl_shape.type = arm_navigation_msgs::Shape::CYLINDER;
    cyl_shape.dimensions.resize(2);
    cyl_shape.dimensions[0] = 0; // radius
    cyl_shape.dimensions[1] = 0; // length

    geometry_msgs::Pose pose;
    tf::Transform tube2cyl, wrist2cyl, tube2wrist = pose2tf(grasp_pose);
    for(size_t i=0; i<cylinders.size(); i++)
    {
        pose = cylinders[i].getLocalPose();
        tube2cyl = pose2tf(pose);
        wrist2cyl = tube2cyl.inverseTimes(tube2wrist);
        pose = tf2pose(wrist2cyl);
        cyl_shape.dimensions[0] = cylinders[i].radius;
        cyl_shape.dimensions[1] = cylinders[i].getAxisLength();
        obj.object.shapes.push_back(cyl_shape);
        obj.object.poses.push_back(pose);
    }
}

void Tube::getCylinderMarker(visualization_msgs::MarkerArray &markerArray)
{
    markerArray.markers.clear();

    visualization_msgs::Marker marker;
    tf::Transform t, tube_tf = getTransform();
    //Axis Marker
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "CylinderAxis";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.color.b = 1;
    marker.color.a = 0.5;
    for(size_t i=0; i<cylinders.size(); i++)
    {
        marker.scale.z = cylinders[i].getAxisLength();
        marker.id = i+1;
        //Assuming Z is cylindrical axis
        //marker.pose = cylinders[i].getGlobalPose();
        t = cylinders[i].getLocalTransform();
        t = tube_tf * t;
        marker.pose = tf2pose(t);
        markerArray.markers.push_back(marker);
    }

    //Cylinder it self
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Cylinder";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.color.r = 0.0;
    marker.color.g = 0.3;
    marker.color.b = 0.3;
    marker.color.a = 0.4;
    for(size_t i=0; i<cylinders.size(); i++)
    {
        marker.scale.x = marker.scale.y = cylinders[i].radius*2;
        marker.scale.z = cylinders[i].getAxisLength();
        marker.id = i+1;
        //Assuming Z is cylindrical axis
        //marker.pose = cylinders[i].getGlobalPose();
        //marker.pose = cylinders[i].getLocalPose();
        t = cylinders[i].getLocalTransform();
        t = tube_tf * t;
        marker.pose = tf2pose(t);
        markerArray.markers.push_back(marker);
    }


    //Cylinder end points
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "CylinderEnds";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.id = 1;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.01;
    for(size_t i=0; i<cylinders.size(); i++)
    {
        marker.pose.position.x = cylinders[i].p1.x;
        marker.pose.position.y = cylinders[i].p1.y;
        marker.pose.position.z = cylinders[i].p1.z;
        markerArray.markers.push_back(marker);
        marker.id++;
        marker.pose.position.x = cylinders[i].p2.x;
        marker.pose.position.y = cylinders[i].p2.y;
        marker.pose.position.z = cylinders[i].p2.z;
        markerArray.markers.push_back(marker);
        marker.id++;
    }
}

void Tube::getCylinderPoses(geometry_msgs::PoseArray &pose_array)
{
    pose_array.poses.clear();
    geometry_msgs::Pose pose;

    pose_array.header.frame_id = "base_link";
    pose_array.header.stamp = ros::Time::now();

    tf::Transform t,tube = getTransform();
    for(size_t i=0; i<cylinders.size(); i++)
    {
        t = cylinders[i].getLocalTransform();
        t = tube * t;
        pose = tf2pose(t);
        pose_array.poses.push_back(pose);
    }
}

geometry_msgs::Pose Cylinder::getLocalPose(void)
{
    return _local_pose;
}

tf::Transform Cylinder::getLocalTransform(void)
{
    tf::Transform t;
    tf::Vector3 v(_local_pose.position.x,
                  _local_pose.position.y,
                  _local_pose.position.z);
    tf::Quaternion q(_local_pose.orientation.x,
                     _local_pose.orientation.y,
                     _local_pose.orientation.z,
                     _local_pose.orientation.w);
    t.setOrigin(v);
    t.setRotation(q);
    return t;
}

void Cylinder::setLocalPose(geometry_msgs::Pose &pose)
{
    _local_pose = pose;
}

void Cylinder::setLocalPose(tf::Transform &t)
{
    tf::Vector3 vec = t.getOrigin();
    tf::Quaternion q = t.getRotation();
    _local_pose.position.x = vec.x();
    _local_pose.position.y = vec.y();
    _local_pose.position.z = vec.z();
    _local_pose.orientation.x = q.x();
    _local_pose.orientation.y = q.y();
    _local_pose.orientation.z = q.z();
    _local_pose.orientation.w = q.w();
}

tf::Vector3 Cylinder::getAxisVector()
{
    tf::Vector3 vec;
    vec.setX(p2.x-p1.x);
    vec.setY(p2.y-p1.y);
    vec.setZ(p2.z-p1.z);
    
    return vec;
}

float Cylinder::getAxisLength()
{
    tf::Vector3 vec = getAxisVector();
    return vec.length();
}

void CloudProcessing::_process_cloud(void)
{
    _compensate_error();
    _estimate_normals();
    _get_radius();
    _collaps_normals();
    _segmentize_axis();
    _define_pose();       // <<<<<<< Cylinders gets converted in local frame including p1 and p2
    _tube->setPoseAsActualPose();
    _generate_work_vectors();
}

//void CloudProcessing::

//random 45/90degree circular trajectory. in global frame
//now in local frame
void CloudProcessing::_generate_work_vectors()
{
    double l = (double)rand()/(double)RAND_MAX; // Not working. Always generates 0.840188
    l = 0.5;
    
    int cyl_idx = rand()%(_tube->cylinders.size()+1);
    //cyl_idx = 1;

    tf::Vector3 axis = _tube->cylinders[cyl_idx].getAxisVector();
    axis.normalize();

    //in global frame
    tf::Vector3 at_point = _tube->cylinders[cyl_idx].getAxisVector();
    at_point *= l;
    at_point.setX(_tube->cylinders[cyl_idx].p1.x+at_point.x());
    at_point.setY(_tube->cylinders[cyl_idx].p1.y+at_point.y());
    at_point.setZ(_tube->cylinders[cyl_idx].p1.z+at_point.z());

    tf::Vector3 perp_vec = _get_perp_vec3(axis);
    tf::Vector3 point, vec1,vec2;

    vec1 = perp_vec.rotate(axis, ((double)rand()/RAND_MAX)*M_PI );
    //vec1 = perp_vec;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    for(int i=0; i<90; i++)
    {
        vec2 = vec1.rotate(axis,(i*M_PI)/180);
        point = at_point + (vec2*_tube->cylinders[cyl_idx].radius);
        vec2.normalize();
        PointT pointnormal;
        pointnormal.x = point.x();
        pointnormal.y = point.y();
        pointnormal.z = point.z();
        pointnormal.normal_x = vec2.x();
        pointnormal.normal_y = vec2.y();
        pointnormal.normal_z = vec2.z();
        cloud->points.push_back(pointnormal);
    }
    _tube->workPointsCluster.push_back(cloud);
}

bool CloudProcessing::writeAxisPointsOnFile(std::string fileName)
{
    if(pcl::io::savePCDFileASCII(fileName,*_tube->axisPoints))
        return true;
    return false;
}

tf::Vector3 CloudProcessing::_get_perp_vec3(tf::Vector3 v3)
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

//Also converts points in local frame (tube frame)
void CloudProcessing::_define_pose(void)
{

    for(size_t i=0; i<_tube->cylinders.size(); i++)
    {
        geometry_msgs::Pose pose;

        pose.position.x = (_tube->cylinders[i].p1.x + _tube->cylinders[i].p2.x) / 2;
        pose.position.y = (_tube->cylinders[i].p1.y + _tube->cylinders[i].p2.y) / 2;
        pose.position.z = (_tube->cylinders[i].p1.z + _tube->cylinders[i].p2.z) / 2;

        tf::Vector3 ux, uy, uz = _tube->cylinders[i].getAxisVector();
        uz.normalize();
        ux = _get_perp_vec3(uz);
        ux.normalize();
        uy = uz.cross(ux);
        uy.normalize();
        tf::Matrix3x3 mat;
        mat.setValue(ux.getX(), uy.getX(), uz.getX(),
                     ux.getY(), uy.getY(), uz.getY(),
                     ux.getZ(), uy.getZ(), uz.getZ() );

        tf::Quaternion q;
        mat.getRotation(q);

        pose.orientation.x = q.getX();
        pose.orientation.y = q.getY();
        pose.orientation.z = q.getZ();
        pose.orientation.w = q.getW();
        //For temporary use only
        _tube->cylinders[i].setLocalPose(pose);
    }

    if(!_tube->cylinders.empty())
    {
        int idx = 0;
        geometry_msgs::Pose pose;

        pose = _tube->cylinders[idx].getLocalPose();
        _tube->setPose(pose);  //Tube's pose = first (strong) cylinder's global pose
    }

    // Convert cylinder from global to local
    for(size_t i=0; i<(_tube->cylinders.size()); i++)
    {
        tf::Transform tube_tf, cyl_tf, tube_cyl_tf;
        cyl_tf = _tube->cylinders[i].getLocalTransform();
        tube_tf = _tube->getTransform();
        tube_cyl_tf = tube_tf.inverseTimes(cyl_tf);
        _tube->cylinders[i].setLocalPose(tube_cyl_tf);
        tf::Transform p1, p2;

        p1.setIdentity();
        p2.setIdentity();
        p1.setOrigin(tf::Vector3( _tube->cylinders[i].p1.x,
                                  _tube->cylinders[i].p1.y,
                                  _tube->cylinders[i].p1.z ) );
        p2.setOrigin(tf::Vector3( _tube->cylinders[i].p2.x,
                                  _tube->cylinders[i].p2.y,
                                  _tube->cylinders[i].p2.z ) );

        p1 = tube_tf.inverseTimes(p1);
        p2 = tube_tf.inverseTimes(p2);

        tf::Vector3 vec = p1.getOrigin();
        _tube->cylinders[i].p1.x = vec.getX();
        _tube->cylinders[i].p1.y = vec.getY();
        _tube->cylinders[i].p1.z = vec.getZ();
        vec = p2.getOrigin();
        _tube->cylinders[i].p2.x = vec.getX();
        _tube->cylinders[i].p2.y = vec.getY();
        _tube->cylinders[i].p2.z = vec.getZ();
    }
}

void CloudProcessing::_compensate_error(void)
{
    for(size_t i=0; i<_tube->tubeCloud->points.size(); i++)
        _tube->tubeCloud->points[i].z -= _z_error;
}

void CloudProcessing::_estimate_normals(void)
{
    pcl::NormalEstimation<PointT, PointT> ne;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(_tube->tubeCloud);

    ne.setRadiusSearch(0.01);
    ROS_INFO("Using approx. view point x=0.0 y=0.0 z=1.5");
    ne.setViewPoint(0.0,0.0,1.5);
    ne.compute(*_tube->tubeCloud);
}

void CloudProcessing::_get_radius(void)
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
    seg.setInputCloud (_tube->tubeCloud);
    seg.setInputNormals (_tube->tubeCloud);
    seg.setDistanceFromOrigin(0.05);
    // Obtain the cylinder inliers and coefficients
    seg.segment (inliers, coeff);
    ROS_INFO("Foud cylinder radius using cylinder RANSAC: %f", coeff.values[6]);
    _r = coeff.values[6]; //_r.push_back(coeff.values[6]);
}

void CloudProcessing::_collaps_normals(void)
{
    _raw_axis_points.reset(new pcl::PointCloud<PointT>);
    _raw_axis_points->header = _tube->tubeCloud->header;
    _raw_axis_points->points.resize(_tube->tubeCloud->points.size());

    for(unsigned int i=0; i<_tube->tubeCloud->points.size(); i++)
    {
        _raw_axis_points->points[i].x = _tube->tubeCloud->points[i].x - _tube->tubeCloud->points[i].normal_x * _r;
        _raw_axis_points->points[i].y = _tube->tubeCloud->points[i].y - _tube->tubeCloud->points[i].normal_y * _r;
        _raw_axis_points->points[i].z = _tube->tubeCloud->points[i].z - _tube->tubeCloud->points[i].normal_z * _r;
    }

    _raw_axis_points->width = _raw_axis_points->points.size();
    _raw_axis_points->height = 1;

    //get_largest_cluster(raw_axis_points, axis_points);
}

void CloudProcessing::_segmentize_axis(void)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    Cylinder cyl;
    PointT p1, p2;

    while(_find_line(inliers,&cyl))
    {
        _tube->cylinders.push_back(cyl);
        _remove_inliers(_raw_axis_points,inliers);

        inliers->indices.clear();

        _cylinder_filter(cyl, _raw_axis_points, inliers);
        _get_line_points(inliers,cyl.coefficients, p1, p2); // no use for p1 & p2
        _remove_inliers(_raw_axis_points,inliers);

        inliers->indices.clear();
    }

    _tube->axisPoints->header = _tube->tubeCloud->header;
    _tube->axisPoints->width = _tube->axisPoints->points.size();
    _tube->axisPoints->height = 1;
    ROS_INFO("%d Axis points found",_tube->axisPoints->points.size());
    ROS_INFO("%d Cylinder found", _tube->cylinders.size());
}

bool CloudProcessing::_find_line(pcl::PointIndices::Ptr inliers, Cylinder *cyl)
{
    if(_raw_axis_points->points.size()< (_min_points*_num_of_points) ) //check if there are enough points to generate model
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
    seg.setInputCloud (_raw_axis_points);
    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers, coeff);

    PointT p1,p2;

    if( inliers->indices.size()>(_min_points*_num_of_points) )
    {
        ROS_INFO("Number of Strong Line inliers : %d",inliers->indices.size());
        ROS_INFO("Line coefficients are: [X= %f Y=%f Z=%f] [N_X=%f N_Y=%f N_Z=%f]", coeff.values[0],coeff.values[1],coeff.values[2],coeff.values[3],coeff.values[4],coeff.values[5]);

        _get_line_points(inliers,coeff, p1, p2);
        cyl->p1 = p1;
        cyl->p2 = p2;
        cyl->isStrong = true;
        cyl->radius = _r;
        cyl->coefficients.header = coeff.header;
        cyl->coefficients.values.resize(7);
        cyl->coefficients.values[0] = p1.x;
        cyl->coefficients.values[1] = p1.y;
        cyl->coefficients.values[2] = p1.z;
        cyl->coefficients.values[3] = p2.x - p1.x;
        cyl->coefficients.values[4] = p2.y - p1.y;
        cyl->coefficients.values[5] = p2.z - p1.z;
        cyl->coefficients.values[6] = _r;
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

    if( inliers->indices.size() > (_min_points*_num_of_points) ) // if confidence in line
    {
        ROS_INFO("Number of Weak Line inliers : %d",inliers->indices.size());
        ROS_INFO("Line coefficients are: [X= %f Y=%f Z=%f] [N_X=%f N_Y=%f N_Z=%f]", coeff.values[0],coeff.values[1],coeff.values[2],coeff.values[3],coeff.values[4],coeff.values[5]);

        _get_line_points(inliers,coeff, p1, p2);
        cyl->p1 = p1;
        cyl->p2 = p2;
        cyl->isStrong = false;
        cyl->radius = _r;
        cyl->coefficients.values.resize(7);
        cyl->coefficients.values[0] = p1.x;
        cyl->coefficients.values[1] = p1.y;
        cyl->coefficients.values[2] = p1.z;
        cyl->coefficients.values[3] = p2.x - p1.x;
        cyl->coefficients.values[4] = p2.y - p1.y;
        cyl->coefficients.values[5] = p2.z - p1.z;
        cyl->coefficients.values[6] = _r;
        
        return true;
    }

    return false;
}

void CloudProcessing::_get_line_points(pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients line_coeff, PointT &p1, PointT &p2)
{
    pcl::PointCloud<PointT>::Ptr line_points(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr line_inliers(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(_raw_axis_points);
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
        _tube->axisPoints->points.push_back(line_points->points[i]);
}

void CloudProcessing::_remove_inliers(pcl::PointCloud<PointT>::Ptr points, pcl::PointIndices::Ptr indices)
{
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(points);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*points);
}

void CloudProcessing::_remove_inliers(pcl::PointCloud<PointT>::Ptr points,  std::vector<int> &indices)
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


void CloudProcessing::_cylinder_filter(Cylinder cyl, pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointIndices::Ptr inliers)
{
    PointT p1, p2;
    p1 = cyl.p1;
    p2 = cyl.p2;
    int pts_cnt = 0;
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;


    float l_sq = (dx*dx) + (dy*dy) + (dz*dz);
    float r_sq = _r*_r;

    for(size_t i=0; i<cloud_in->points.size(); i++)
    {
        if(isInCylinder(p1, p2, l_sq, r_sq, cloud_in->points[i])>0)
        {
            inliers->indices.push_back(i);
            pts_cnt++;
        }
    }
    ROS_INFO("%d Points found in cylinder filter",pts_cnt);
}

void CloudProcessing::setZerror(float error)
{
    _z_error = error;
}

/*void CloudProcessing::displayCloud(void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (_tube->tubeCloud,"tube_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tube_cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->spin();
}
void CloudProcessing::displayAxisPoints(void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (_tube->axisPoints,"axis");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "axis");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->spin();
}

void CloudProcessing::displayCylinders(void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::ModelCoefficients coeffs;
    //float sum;
    coeffs.values.resize(7);
    for (size_t i=0; i<_tube->cylinders.size(); i++)
    {
        coeffs.header = _tube->cylinders[i].coefficients.header;
        coeffs.values[0] = _tube->cylinders[i].p1.x;
        coeffs.values[1] = _tube->cylinders[i].p1.y;
        coeffs.values[2] = _tube->cylinders[i].p1.z;
        coeffs.values[3] = _tube->cylinders[i].p2.x-_tube->cylinders[i].p1.x;
        coeffs.values[4] = _tube->cylinders[i].p2.y-_tube->cylinders[i].p1.y;
        coeffs.values[5] = _tube->cylinders[i].p2.z-_tube->cylinders[i].p1.z;
        coeffs.values[6] = _tube->cylinders[i].radius;
        std::strstream ss;
        ss<<"Cylinder_"<<i;
        viewer->addCylinder(coeffs,ss.str());
    }
    viewer->addPointCloud<PointT> (_tube->axisPoints,"LinePoints");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "LinePoints");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->spin();
}

void CloudProcessing::displayCylinders(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
     //(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //viewer->setBackgroundColor (0, 0, 0);
    pcl::ModelCoefficients coeffs;
    //float sum;
    coeffs.values.resize(7);
    for (size_t i=0; i<_tube->cylinders.size(); i++)
    {
        coeffs.header = _tube->cylinders[i].coefficients.header;
        coeffs.values[0] = _tube->cylinders[i].p1.x;
        coeffs.values[1] = _tube->cylinders[i].p1.y;
        coeffs.values[2] = _tube->cylinders[i].p1.z;
        coeffs.values[3] = _tube->cylinders[i].p2.x-_tube->cylinders[i].p1.x;
        coeffs.values[4] = _tube->cylinders[i].p2.y-_tube->cylinders[i].p1.y;
        coeffs.values[5] = _tube->cylinders[i].p2.z-_tube->cylinders[i].p1.z;
        coeffs.values[6] = _tube->cylinders[i].radius;
        std::strstream ss;
        ss<<"Cylinder_"<<i;
        viewer->addCylinder(coeffs,ss.str());
    }
    viewer->addPointCloud<PointT> (_tube->axisPoints,"LinePoints");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "LinePoints");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->spin();
}

void CloudProcessing::displayCylindersInLocalFrame(void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::ModelCoefficients coeffs;
    geometry_msgs::Pose pose;
    tf::Transform tf;
    tf::Matrix3x3 mat;
    tf::Vector3 colm1,colm2,colm3, vec;
    tf::Quaternion q;

    coeffs.values.resize(7);
    for (size_t i=0; i<_tube->cylinders.size(); i++)
    {
        coeffs.header = _tube->cylinders[i].coefficients.header;
        pose = _tube->cylinders[i].getLocalPose();
        vec = _tube->cylinders[i].getAxisVector();

        coeffs.values[0] = pose.position.x;
        coeffs.values[1] = pose.position.y;
        coeffs.values[2] = pose.position.z;
        coeffs.values[3] = _tube->cylinders[i].p2.x-_tube->cylinders[i].p1.x;
        coeffs.values[4] = _tube->cylinders[i].p2.y-_tube->cylinders[i].p1.y;
        coeffs.values[5] = _tube->cylinders[i].p2.z-_tube->cylinders[i].p1.z;
        coeffs.values[6] = _tube->cylinders[i].radius;
        std::strstream ss;
        ss<<"Cylinder_"<<i;
        viewer->addCylinder(coeffs,ss.str());
    }
    viewer->addPointCloud<PointT> (_tube->axisPoints,"LinePoints");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "LinePoints");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->spin();
}

void CloudProcessing::displayLines(void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::ModelCoefficients coeffs;
    coeffs.header = _tube->cylinders[0].coefficients.header;
    coeffs.values.resize(6);
    for (size_t i=0; i<_tube->cylinders.size(); i++)
    {
        std::strstream ss;
        ss<<"Line_"<<i;
        viewer->addLine(_tube->cylinders[i].p1,_tube->cylinders[i].p2,ss.str());
    }
    //viewer->addPointCloud<PointT> (_tube->axisPoints,"LinePoints");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "LinePoints");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->spin();
}

void CloudProcessing::dispalyWorkTraj(void)
{
    pcl::PointCloud<PointT>::Ptr cloud;//(new pcl::PointCloud<PointT>);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> 
            viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    for(int i=0; i<_tube->workPointsCluster.size(); i++)
    {
        cloud = _tube->workPointsCluster[i];

        std::strstream ss;
        ss.flush();
        ss<<"C"<<i;
        viewer->addPointCloud<PointT>(cloud,ss.str());
    }
    
    
    viewer->setBackgroundColor (0, 0, 0);
    //viewer->addPointCloudNormals<PointT,pcl::PointNormal> (cloud,pcl_normals);
    viewer->addPointCloud<PointT> (_tube->tubeCloud,"tube_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tube_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "C0");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->spin();
}*/

}//TubePerception

