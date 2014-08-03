#include "tubePerception.h"

#define PRCPN_LGRNM "prcp"


namespace TubePerception
{

//! \brief Returns local pose of cylinder
geometry_msgs::Pose Cylinder::getPose(void){
    return _local_pose;
}

//! \brief Returns local pose of cylinder as tf::Transform object
tf::Transform Cylinder::getTransform(void){
    return pose2tf(_local_pose);
}

//! \brief Sets local pose of cylinder
void Cylinder::setPose(const geometry_msgs::Pose &pose){
    _local_pose = pose;
}

//! \brief Sets local pose of cylinder
void Cylinder::setPose(const tf::Transform &t){
    _local_pose = tf2pose(t);
}

//! \brief returns global pose of cylinder given pose of tube this cylinder part of
geometry_msgs::Pose Cylinder::getGlobalPose(geometry_msgs::Pose &tubePose){
    tf::Transform tubeTf = pose2tf(tubePose);
     return (tf2pose(getGlobalTf(tubeTf)));
}

//! \brief Returns local pose of cylinder
tf::Transform Cylinder::getGlobalTf(tf::Transform &tubeTF){
    tf::Transform local_tf = pose2tf(_local_pose);
    return (tubeTF * local_tf);
}

//! \brief Returns axis of cylinder in gloabl frame given pose in transform object of tube this cylinder part of
tf::Vector3 Cylinder::getAxisVector(tf::Transform &tubeTf){
    tf::Vector3 new_p1, new_p2;
    _convert_points_in_global(tubeTf, new_p1,new_p2);
    tf::Vector3 vec = new_p2 - new_p1;
    return vec;
}

//! \brief Returns length of cylinder
double Cylinder::getAxisLength(){
    tf::Vector3 vec = p2-p1;
    return vec.length();
}


//! \brief Returns mid point of cylinder in globale frame given the pose of tube this clyinder part of
tf::Vector3 Cylinder::getMidPoint(tf::Transform &tubeTf){
    tf::Vector3 new_p1, new_p2;
    _convert_points_in_global(tubeTf, new_p1, new_p2);
    tf::Vector3 point = new_p1 + new_p2;
    point /= 2;
    return point;
}

//! \brief Returns first point of cylinder's axis in global frame given the pose of tube
tf::Vector3 Cylinder::getGlobalP1(tf::Transform &tubeTf){
    tf::Transform point_tf;
    point_tf.setIdentity();
    point_tf.setOrigin(p1);
    point_tf = tubeTf * point_tf;
    return point_tf.getOrigin();
}

//! \brief Returns local pose of cylinder
 tf::Vector3 Cylinder::getGlobalP2(tf::Transform &tubeTf){
    tf::Transform point_tf;
    point_tf.setIdentity();
    point_tf.setOrigin(p2);
    point_tf = tubeTf * point_tf;
    return point_tf.getOrigin();
}

 //! \brief Given transform, convert's first point of cylinder in transform's source frame
void Cylinder::_convert_points_in_global(tf::Transform &tf, tf::Vector3 &p1_out, tf::Vector3 &p2_out){
    tf::Transform point_tf;
    point_tf.setIdentity();
    point_tf.setOrigin(p1);
    point_tf = tf * point_tf;
    p1_out = point_tf.getOrigin();
    point_tf.setOrigin(p2);
    point_tf = tf * point_tf;
    p2_out = point_tf.getOrigin();
}

/*! \brief Tests if @param testPoint (in global frame) is inside cylinder
* inflates cylinder by adding 1mm in radius
* Uses global p1 and p2 of cylinder
*/
bool Cylinder::isInlier(tf::Vector3 &testPoint, tf::Transform &tubeTf){
    tf::Vector3 p1_new, p2_new;
    _convert_points_in_global(tubeTf, p1_new, p2_new);
    tf::Vector3 vec = p2_new - p1_new;
    double r = radius + 0.001;
    return (isInCylinder(p1_new, p2_new, vec.length2(), r*r, testPoint));
}

//! \brief Tube Object Constructor.
Tube::Tube(){

}

//! \brief Sets the pose of tube
void Tube::setPose(geometry_msgs::Pose &pose){
    _pose = pose;
}

//! \brief Returns set pose of tube
geometry_msgs::Pose Tube::getPose(void){
    return _pose;
}

//! \brief Returns set pose of tube in tf::Transform object
tf::Transform Tube::getTransform(void){
    tf::Transform tf = pose2tf(_pose);
    return tf;
}

//! \brief Returns the global pose of cylinder given the index of cylinders of tube.
//! std lib throws a runtime error if cylIdx is out of range
geometry_msgs::Pose Tube::getCylinderGlobalPose(unsigned int cylIdx){
    if(cylinders.empty())
        ROS_ERROR_NAMED(PRCPN_LGRNM,"No cylinder present. Illegal query for pose");
    tf::Transform c, t = this->getTransform();
    TubePerception::Cylinder cyl = this->cylinders.at(cylIdx);
    c = cyl.getTransform();
    c = t*c;
    return tf2pose(c);
}

void Tube::setPoseAsActualPose(){
    _actual_pose = _pose;
}
void Tube::resetPoseToActual(){
    _pose = _actual_pose;
}

// graspPose is in local to tube frame, wristPose is in global. pose of arm holding tube.
/*void Tube::resetActualPose(geometry_msgs::Pose &graspPose, geometry_msgs::Pose &wristPose){
    tf::Transform g = pose2tf(graspPose), w = pose2tf(wristPose), tube;
    tube = w * g.inverse();
    _pose = tf2pose(tube);
    setPoseAsActualPose();
}*/


/*! \brief Given a point in local (tube) frame, tests whether it lies inside cylinder
 *  inflates cylider by adding 0.002 in radius.
 */
unsigned int Tube::whichCylinder(PointT localPoint)
{
    //ROS_INFO_STREAM_NAMED(PRCPN_LGRNM,"Test point : \n"<<localPoint);

    //convert in to vector3
    tf::Vector3 p;
    p.setValue(localPoint.x, localPoint.y, localPoint.z);

    //identity
    tf::Vector3 vec;
    double r;
    for(size_t i=0; i<cylinders.size(); i++){
        vec = cylinders[i].p2 - cylinders[i].p1;
        r = cylinders[i].radius;
        r += 0.002; //inflate cylinder
        if(isInCylinder(cylinders[i].p1, cylinders[i].p2, vec.length2(), r*r, p)){
            return i;
        }
    }
    return cylinders.size();
}

//! \brief 
void Tube::getAttachedObjForRightGrasp(geometry_msgs::Pose right_grasp_pose,
                                  arm_navigation_msgs::AttachedCollisionObject::Ptr objPtr){
    _get_attached_collision_object(objPtr, right_grasp_pose, "r_wrist_roll_link", true, false);
}

void Tube::getAttachedObjForLeftGrasp(geometry_msgs::Pose left_grasp_pose,
                                 arm_navigation_msgs::AttachedCollisionObject::Ptr objPtr){
    _get_attached_collision_object
            (objPtr, left_grasp_pose, "l_wrist_roll_link", false, true);
}

void Tube::getAttachedObjForBothGrasps(geometry_msgs::Pose right_grasp_pose,
                                  arm_navigation_msgs::AttachedCollisionObject::Ptr objPtr){
    _get_attached_collision_object
            (objPtr, right_grasp_pose, "r_wrist_roll_link", true, true);
}

//from grasp to tube
void Tube::_get_attached_collision_object(arm_navigation_msgs::AttachedCollisionObject::Ptr obj_ptr,
                                          geometry_msgs::Pose &grasp_pose,
                                          std::string link_name,
                                          bool right_side, bool left_side)
{
    obj_ptr->link_name = link_name.c_str();
    obj_ptr->object.header.frame_id = link_name.c_str();

    if(link_name.compare("r_wrist_roll_link")==0)
        right_side = true;
    if(link_name.compare("l_wrist_roll_link")==0)
        left_side = true;

    obj_ptr->touch_links.clear();
    if(right_side){
        //obj_ptr->touch_links.push_back("r_end_effector");
        obj_ptr->touch_links.push_back("r_gripper_palm_link");
        obj_ptr->touch_links.push_back("r_gripper_l_finger_link");
        obj_ptr->touch_links.push_back("r_gripper_l_finger_tip_link");
        obj_ptr->touch_links.push_back("r_gripper_motor_accelerometer_link");
        obj_ptr->touch_links.push_back("r_gripper_led_frame");
        obj_ptr->touch_links.push_back("r_gripper_motor_slider_link");
        obj_ptr->touch_links.push_back("r_gripper_motor_screw_link");
        obj_ptr->touch_links.push_back("r_gripper_r_finger_link");
        obj_ptr->touch_links.push_back("r_gripper_r_finger_tip_link");
        obj_ptr->touch_links.push_back("r_gripper_l_finger_tip_frame");
        obj_ptr->touch_links.push_back("r_gripper_tool_frame");
    }
    if(left_side){
        //obj_ptr->touch_links.push_back("l_end_effector");
        obj_ptr->touch_links.push_back("l_gripper_palm_link");
        obj_ptr->touch_links.push_back("l_gripper_l_finger_link");
        obj_ptr->touch_links.push_back("l_gripper_l_finger_tip_link");
        obj_ptr->touch_links.push_back("l_gripper_motor_accelerometer_link");
        obj_ptr->touch_links.push_back("l_gripper_led_frame");
        obj_ptr->touch_links.push_back("l_gripper_motor_slider_link");
        obj_ptr->touch_links.push_back("l_gripper_motor_screw_link");
        obj_ptr->touch_links.push_back("l_gripper_r_finger_link");
        obj_ptr->touch_links.push_back("l_gripper_r_finger_tip_link");
        obj_ptr->touch_links.push_back("l_gripper_l_finger_tip_frame");
        obj_ptr->touch_links.push_back("l_gripper_tool_frame");
    }

    obj_ptr->object.id = "tube";
    obj_ptr->object.header.stamp = ros::Time::now();
    obj_ptr->object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

    arm_navigation_msgs::Shape cyl_shape;
    cyl_shape.type = arm_navigation_msgs::Shape::CYLINDER;
    cyl_shape.dimensions.resize(2);
    cyl_shape.dimensions[0] = 0; // radius
    cyl_shape.dimensions[1] = 0; // length
    obj_ptr->object.padding = 1.1;

    obj_ptr->object.shapes.clear();
    obj_ptr->object.poses.clear();
    geometry_msgs::Pose pose;
    tf::Transform tube2cyl, wrist2cyl, tube2wrist = pose2tf(grasp_pose);
    for(size_t i=0; i<cylinders.size(); i++){
        pose = cylinders[i].getPose();
        tube2cyl = pose2tf(pose);
        //wrist2cyl = tube2cyl.inverseTimes(tube2wrist);
        wrist2cyl = tube2wrist.inverseTimes(tube2cyl);
        pose = tf2pose(wrist2cyl);
        cyl_shape.dimensions[0] = cylinders[i].radius;
        cyl_shape.dimensions[1] = cylinders[i].getAxisLength();
        obj_ptr->object.shapes.push_back(cyl_shape);
        obj_ptr->object.poses.push_back(pose);
    }
}

void Tube::getCollisionObject(arm_navigation_msgs::CollisionObject &obj)
{
    obj.header.frame_id = "/base_link";
    obj.header.stamp = ros::Time::now();
    obj.id = "static_tube";
    obj.operation.operation = obj.operation.ADD;
    //obj.padding = 1.0;
    arm_navigation_msgs::Shape cyl_shape;
    cyl_shape.type = arm_navigation_msgs::Shape::CYLINDER;
    cyl_shape.dimensions.resize(2);
    cyl_shape.dimensions[0] = 0; // radius
    cyl_shape.dimensions[1] = 0; // length

    obj.shapes.clear();
    obj.poses.clear();
    geometry_msgs::Pose pose;
    for(size_t i=0; i<cylinders.size(); i++){
        pose = cylinders[i].getGlobalPose(_pose);
        cyl_shape.dimensions[0] = cylinders[i].radius;
        cyl_shape.dimensions[1] = cylinders[i].getAxisLength();
        obj.shapes.push_back(cyl_shape);
        //ROS_WARN_NAMED(PRCPN_LGRNM,"***Pose.position = %f   %f   %f",pose.position.x, pose.position.y,pose.position.z);
        obj.poses.push_back(pose);
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
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.a = 1;
    marker.lifetime = ros::Duration(15);
    for(size_t i=0; i<cylinders.size(); i++){
        marker.scale.z = cylinders[i].getAxisLength();
        marker.id = i+1;
        //Assuming Z is cylindrical axis
        marker.pose = cylinders[i].getGlobalPose(_pose);
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
    marker.color.a = 0.7;
    //marker.lifetime = ros::Duration(20);
    for(size_t i=0; i<cylinders.size(); i++){
        marker.scale.x = marker.scale.y = cylinders[i].radius*2;
        marker.scale.z = cylinders[i].getAxisLength();
        marker.id = i+1;
        //Assuming Z is cylindrical axis
        //marker.pose = cylinders[i].getGlobalPose();
        //marker.pose = cylinders[i].getPose();
        marker.pose = cylinders[i].getGlobalPose(_pose);
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
    marker.color.a = 0.5;
    marker.id = 1;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.005;
    marker.lifetime = ros::Duration(15);
    for(size_t i=0; i<cylinders.size(); i++){

        tf::Transform p;
        p.setIdentity();
        p.setOrigin(cylinders[i].p1);
        p = tube_tf * p;
        tf::Vector3 vec = p.getOrigin();

        marker.pose.position.x = vec.getX();
        marker.pose.position.y = vec.getY();
        marker.pose.position.z = vec.getZ();
        markerArray.markers.push_back(marker);

        p.setIdentity();
        p.setOrigin(cylinders[i].p2);
        p = tube_tf * p;
        vec = p.getOrigin();

        marker.id++;
        marker.pose.position.x = vec.getX();
        marker.pose.position.y = vec.getY();
        marker.pose.position.z = vec.getZ();
        markerArray.markers.push_back(marker);
        marker.id++;
    }
}

void Tube::getCylinderPoses(geometry_msgs::PoseArray &pose_array){
    pose_array.poses.clear();
    geometry_msgs::Pose pose;
    pose_array.header.frame_id = "base_link";
    pose_array.header.stamp = ros::Time::now();

    for(size_t i=0; i<cylinders.size(); i++){
        pose = cylinders[i].getGlobalPose(_pose);
        pose_array.poses.push_back(pose);
    }
}

void Tube::getWorkPointsMarker(visualization_msgs::MarkerArray &markerArray){
    pcl::PointCloud<PointT>::Ptr cloud;
    visualization_msgs::Marker marker;
    marker.action = marker.ADD;
    marker.id = 0;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.type = marker.ARROW;
    marker.lifetime = ros::Duration(15);
    /*marker.color.a = 0.5;
    marker.color.r = 1;
    marker.color.b = 0;*/
    geometry_msgs::Point point, p_global;
    tf::Vector3 norm_vec;
    marker.scale.x = 0.001;
    marker.scale.y = 0.0015;
    marker.scale.z = 0.003;
    tf::Transform tube_tf = pose2tf(_pose), point_tf;
    tf::Vector3 vec;
    point_tf.setIdentity();
    std_msgs::ColorRGBA clr;
    clr.a = 0.5;
    clr.r = 1;
    clr.b = 0;
    double clr_step = 1;
    for(unsigned int i=0; i<workPointsCluster.size(); i++){
        cloud = workPointsCluster[i];
        for(unsigned int j=0; j<cloud->points.size(); j++){
            marker.id++;
            marker.color = clr;
            point.x = cloud->points[j].x;
            point.y = cloud->points[j].y;
            point.z = cloud->points[j].z;
            norm_vec.setValue(cloud->points[j].normal_x,
                          cloud->points[j].normal_y,
                          cloud->points[j].normal_z);
            norm_vec.normalize();
            norm_vec *= 0.02; // 20mm long
            //
            point_tf.setOrigin(tf::Vector3(point.x, point.y, point.z));
            point_tf = tube_tf * point_tf;
            vec = point_tf.getOrigin();
            p_global.x = vec.getX();
            p_global.y = vec.getY();
            p_global.z = vec.getZ();
            marker.points.push_back(p_global);

            point.x += norm_vec.getX();
            point.y += norm_vec.getY();
            point.z += norm_vec.getZ();
            //
            point_tf.setOrigin(tf::Vector3(point.x, point.y, point.z));
            point_tf = tube_tf * point_tf;
            vec = point_tf.getOrigin();
            p_global.x = vec.getX();
            p_global.y = vec.getY();
            p_global.z = vec.getZ();
            marker.points.push_back(p_global);

            markerArray.markers.push_back(marker);
            marker.points.clear();
        }

        clr.r -=clr_step;
        if(clr.r<0){
            clr.r=1;
        }

        clr.b +=clr_step;
        if(clr.b>1){
            clr.b=0;
        }
    }
}

void Tube::reset(){
    cylinders.clear();
    workPointsCluster.clear();
} // Class Tube


CloudProcessing::CloudProcessing()
{
    //default params
    _r = 0;
    _r_min = 0.01;
    _r_max = 0.05;
    _strong_line_thr = 0.2; // strong cylinder formed by at least 20% of points in cloud
    _weak_line_thr = 0.05;  // cylinder is weak if there are only 5% of points to define line
                            // if less than 5% inliers of line do not define cylinder
    _min_points = 0.05; //in fraction ex. 1/20 times total points
    _z_error = 0;
}

bool CloudProcessing::genTubeModel(const sensor_msgs::PointCloud2 &clusterCloud, Tube::Ptr tube_ptr)
{
    //copy cloud in pcl type and initialize pointers
    _tube_cloud.reset(new pcl::PointCloud<PointT>);
    _axis_points.reset(new pcl::PointCloud<PointT>);
    //_convert_cloud_to("/base_link",)
    _convert_to_pcl(clusterCloud, _tube_cloud);
    //writePointCloudOnfile(clusterCloud,"clusterCloud1.pcd");
    //writePointCloudOnfile(_tube_cloud,"clusterCloud2.pcd");
    pcl::PointCloud<PointT>::Ptr points_of_interest(new pcl::PointCloud<PointT>);
    _get_points_of_interest(_tube_cloud, points_of_interest);

    //displayCloud2(interest_points_cloud);
    _tube = tube_ptr;
    _tube->reset();
    _num_of_points = _tube_cloud->points.size();
    std::cout<<'\n'<<"Initial_cloud"<<'\n';
    displayCloud(_tube_cloud,"Initial_cloud");
    _compensate_error();
    _estimate_normals(_tube_cloud);
    std::cout<<'\n'<<"Initial_cloud_with_normals"<<'\n';
    displayCloudWithNormals(_tube_cloud,"Initial_cloud_with_normals");
    _r = _get_radius();
    if(_r==0){
        return false;
    }
    _collaps_normals();
    std::cout<<'\n'<<"raw_axis"<<'\n';
    displayCloud(_raw_axis_points,"raw_axis");
    _segmentize_axis();
    _define_pose(); // <<< Cylinders gets converted in local frame including p1 and p2
    _tube->setPoseAsActualPose();
    if(_tube->cylinders.empty()){
        _tube->reset();
        ROS_WARN_NAMED(PRCPN_LGRNM,"No cylinder found in cluster!");
        return false;
    }
    //_generate_work_vectors();  // could be plugin for rgb processing
    _generate_work_vectors(points_of_interest);
    //_tube->workPointsCluster.push_back(points_of_interest);
    return true;
}

void CloudProcessing::_get_points_of_interest(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<PointT>::Ptr cloud_out){
    if(cloud_in->points.empty()){
        return;
    }

    PointT point;
    //std::vector<unsigned int> red_points;
    pcl::PointIndices::Ptr interest_points(new pcl::PointIndices);
    for(unsigned int i=0; i<cloud_in->points.size(); i++){
        point = cloud_in->points[i];
        //extract red green and blue
        uint32_t rgb = *reinterpret_cast<int*>(&point.rgb);
        uint8_t r = (rgb >> 16) & 0x0000ff;
        uint8_t g = (rgb >> 8) & 0x0000ff;
        uint8_t b = (rgb) & 0x0000ff;
        unsigned int intensity = r + b + g;
         if(intensity <255){ //if intensity is less 33.33%
            interest_points->indices.push_back(i);
         }
    }
    interest_points->header.frame_id="/base_link";
    interest_points->header.stamp = ros::Time::now();
    ROS_WARN_NAMED(PRCPN_LGRNM, "%u interest points",interest_points->indices.size());
    pcl::ExtractIndices<PointT> extract;
    extract.setIndices(interest_points);
    extract.setInputCloud(cloud_in);
    extract.setNegative(false);
    extract.filter(*cloud_out);
}

bool CloudProcessing::resetPoseOfTube(const sensor_msgs::PointCloud2 &cluster, TubePerception::Tube::Ptr tube_ptr){
    //copy cloud in pcl type and initialize pointers
    _tube_cloud.reset(new pcl::PointCloud<PointT>);
    _axis_points.reset(new pcl::PointCloud<PointT>);
    //_convert_cloud_to("/base_link",)
    _convert_to_pcl(cluster, _tube_cloud);
    _tube.reset(new TubePerception::Tube);
    _num_of_points = _tube_cloud->points.size();

    //_compensate_error();
    _estimate_normals(_tube_cloud);
    _r = _get_radius();
    _collaps_normals();
    _segmentize_axis();
    _define_pose(); // <<< Cylinders gets converted in local frame including p1 and p2
    if(_tube->cylinders.empty()){
        _tube->reset();
        ROS_WARN_NAMED(PRCPN_LGRNM,"No cylinder found in cluster!");
        return false;
    }

    std::vector<unsigned int> corres_ind;
    std::vector<double> confidence;
    _compare_models(tube_ptr, _tube, corres_ind, confidence );
    //since the pose of any tube model is first cylinder
    // find fisrt cylinder
    ROS_INFO_NAMED(PRCPN_LGRNM,"Existing ->  New");
    int corresp_cyl_cnt = 0;
    for(unsigned int i=0; i<corres_ind.size(); i++){
        if(corres_ind[i]<corres_ind.size()){
            corresp_cyl_cnt++;
        }
        ROS_INFO_NAMED(PRCPN_LGRNM,"     %d   ->   %d",i,corres_ind[i]);
    }

    if(corresp_cyl_cnt<2){
        ROS_WARN_NAMED(PRCPN_LGRNM,"Couldn't find enough corresponding cylinders");
        return false;
    }
    //assuming there will be corresponding first cylinder in new tube
    if(corres_ind[0]<_tube->cylinders.size()){

        tf::Transform temp_tf, first_tf, second_tf;

        tf::Vector3 first_orig, second_orig;

        first_tf = tube_ptr->getTransform();
        //second_tf = _tube->getTransform();
        first_orig = tube_ptr->cylinders[0].getMidPoint(first_tf);
        second_tf = _tube->getTransform();
        second_orig = _tube->cylinders[corres_ind[0]].getMidPoint(second_tf);
        second_tf.setRotation(first_tf.getRotation());
        second_tf.setOrigin(second_orig);
        tf::Transform err_tf1 = first_tf.inverseTimes(second_tf);
        err_tf1.setRotation(tf::Quaternion(0,0,0,1));

        temp_tf = tube_ptr->getTransform();
        tf::Vector3 axis_first = tube_ptr->cylinders[0].getAxisVector(temp_tf);
        temp_tf = _tube->getTransform();
        tf::Vector3 axis_second = _tube->cylinders[corres_ind[0]].getAxisVector(temp_tf);
        //double angle = axis_first.angle(axis_second);
        //tf::Vector3 perp_vec = axis_first.cross(axis_second);
        //err_tf1.setRotation(tf::Quaternion(perp_vec, angle));

        tf::Transform err_tf2;
        err_tf2.setIdentity();

        // at this point z axis of both tube and origin is aligned

        // now align y and x by minimizing error by rotating around z axis
        unsigned int cnt = 1000;
        tf::Transform tube_tf = tube_ptr->getTransform(), new_tube_tf = _tube->getTransform();
        tf::Transform tube_plus_err;
        tf::Vector3 vec;
//        std::string s;
//        std::cin>>s;

        double theta_step = (2*M_PI)/cnt;
        double theta = 0;
        double prev_err = 0;
        double err = 0;
        //start with theta = 0
        while(cnt>0){
            err = 0;
            err_tf2.setIdentity();
            err_tf2.setRotation(tf::Quaternion(tf::Vector3(0, 0, 1), theta));
            tube_plus_err = tube_tf * err_tf1 * err_tf2;
            for(unsigned int i=0; i<tube_ptr->cylinders.size(); i++){
                if(corres_ind[i]<corres_ind.size()){
                    vec = tube_ptr->cylinders[i].getGlobalP1(tube_plus_err) -
                            _tube->cylinders[corres_ind[i]].getGlobalP1(new_tube_tf);
                    err += vec.length();
                    vec = tube_ptr->cylinders[i].getGlobalP2(tube_plus_err) -
                            _tube->cylinders[corres_ind[i]].getGlobalP2(new_tube_tf);
                    err += vec.length();
                }
            }
            if(err>prev_err){
                theta -= theta_step;
            }
            else{
                theta += theta_step;
            }
            //rotation bounds
            if(theta>2*M_PI){
                theta -= (2*M_PI);
            }
            if(theta<0){
                theta += (2*M_PI);
            }
            //std::cout<<"\ntheta : "<<theta<<"err : "<<err;
            prev_err = err;
            cnt--;
        }
        err_tf2.setIdentity();
        //err_tf2.setRotation(tf::Quaternion(tf::Vector3(0, 0, 1), theta));
        temp_tf = tube_ptr->getTransform();
        temp_tf = temp_tf * err_tf1 * err_tf2;
        tf::Vector3 pos_err = err_tf1.getOrigin();
        ROS_INFO_NAMED(PRCPN_LGRNM,"Tube position Error : x=%f, y=%f, z=%f",
                              pos_err.getX(), pos_err.getY(), pos_err.getZ());
        geometry_msgs::Pose pose = tf2pose(temp_tf);
        tube_ptr->setPose(pose);
        tube_ptr->setPoseAsActualPose();
    }
    else{
        ROS_WARN_NAMED(PRCPN_LGRNM,"Couldn't find first cylinder in new model!");
        return false;
    }
    return true;
}

// Under construction... to be extended to pose inverient feature comparision.
void CloudProcessing::_compare_models(TubePerception::Tube::Ptr first,
                                      TubePerception::Tube::Ptr second,
                                      std::vector<unsigned int> & corresponding_indices,
                                      std::vector<double> &confidence){
    const double W_l = 0.5;
    const double W_t = 0.5;

    corresponding_indices.resize(first->cylinders.size());
    confidence.resize(first->cylinders.size());
    unsigned int idx = std::numeric_limits<unsigned int>::max();
    tf::Transform temp_tf;
    // compare distance between  two mid points and store the index of closest one
    for(unsigned int i=0; i<first->cylinders.size(); i++){

        temp_tf = first->getTransform();
        tf::Vector3 first_axis = first->cylinders[i].getAxisVector(temp_tf), second_axis;
        double max_confidence = std::numeric_limits<double>::max(),
                first_len, second_len, len_err, angle1, angle2, conf;
        first_len = first->cylinders[i].getAxisLength();
        //compare mid points/axis length/axis vector of the cylinders in second tube and store index id distance is less than certain value (radius)
        for(unsigned int j=0; j<second->cylinders.size(); j++){
            temp_tf = second->getTransform();
            second_axis = second->cylinders[j].getAxisVector(temp_tf);
            second_len = second->cylinders[j].getAxisLength();
            angle1 = first_axis.angle(second_axis);
            second_axis *= -1;
            angle2 = first_axis.angle(second_axis);
            angle1 = std::abs(angle1);
            angle2 = std::abs(angle2);
            if(angle2<angle1){
                angle1 = angle2;
            }
            len_err = std::abs(first_len - second_len);
            conf = len_err*W_l + angle1*W_t;
            //std::cout<<"\nconfidence : "<<conf;
            if(conf<max_confidence){
                max_confidence = conf;
                idx = j;
            }
        }
        //std::cout<<"\n";
        corresponding_indices[i] = idx;
        confidence[i] = max_confidence;
    }
}

/*! \brief Given tube model and scene point cloud, extracts points posibally part of known tube.
  * tests all points in @param cloudIn, if they are part of any cylinder of given tube.
  */

bool CloudProcessing::extractTubePoints(Tube::Ptr tube, sensor_msgs::PointCloud2ConstPtr cloudIn, sensor_msgs::PointCloud2::Ptr cloudOut)
{
    if(tube->cylinders.empty()){
        ROS_WARN_NAMED(PRCPN_LGRNM,"No cylinder in tube model!");
        return false;
    }
    double r_inflate_coeff = 1.3; // inflate radius of cylinder by 1.3 times
    double l_inflate_coeff = 1.2; // 20% longer axis
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2 cloudIn_converted;
    //converts cloud in base_link if not already converted
    _convert_cloud_to("/base_link",*cloudIn,cloudIn_converted);
    //_convert_to_pcl(cloudIn_converted, cloud_in);
    pcl::fromROSMsg(cloudIn_converted, *cloud_in);

    //container to be used to test points
    std::vector<double> r_sq(tube->cylinders.size());
    std::vector<double> l_sq(tube->cylinders.size());
    std::vector<tf::Vector3> p1(tube->cylinders.size());
    std::vector<tf::Vector3> p2(tube->cylinders.size());
    tf::Vector3 point1,point2,mid_point, axis;
    double len;
    tf::Transform tube_tf = tube->getTransform();
    for(unsigned int i=0; i<tube->cylinders.size(); i++){
        r_sq[i] = tube->cylinders[i].radius * r_inflate_coeff;
        r_sq[i] = r_sq[i] * r_sq[i];
        len = tube->cylinders[i].getAxisLength() * l_inflate_coeff;
        l_sq[i] = len * len;
        mid_point = tube->cylinders[i].getMidPoint(tube_tf);
        axis = tube->cylinders[i].getAxisVector(tube_tf);
        axis.normalize();
        axis *= len/2;
        point2 = mid_point + axis;
        axis *= (-1);
        point1 = mid_point + axis;

        p1[i] = point1;
        p2[i] = point2;
    }
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
    cloud_out->points.clear();
    tf::Vector3 test_point;

    // for all points and all cylinder in tube, test if point belongs to any cylinder
    for(unsigned int i=0; i<cloud_in->points.size(); i++){
        test_point.setX(cloud_in->points[i].x);
        test_point.setY(cloud_in->points[i].y);
        test_point.setZ(cloud_in->points[i].z);
        //for each cylinder check if land inside cylinder
        for(unsigned int j=0; j<tube->cylinders.size(); j++){
            //if lands inside cylinder add point to cloud
            if(isInCylinder(p1[j], p2[j], l_sq[j], r_sq[j], test_point)){
                cloud_out->points.push_back(cloud_in->points[i]);
            }
        }
    }

    pcl::toROSMsg(*cloud_out,*cloudOut);
    ROS_INFO_NAMED(PRCPN_LGRNM, "Total %d points extracted from cloud", (int)cloud_out->points.size());

    //displayCloud(cloud_in);
    //displayCloud(cloud_out);
    return true;
}

/*! \brief finds wheel of given radius limits
  * not to be used for general purpose. specific to 3d model of grinder machine body
  */
bool CloudProcessing::findDisk(const sensor_msgs::PointCloud2 &clusterCloud,
                               double minRadius, double maxRadius,
                               TubePerception::Cylinder &disk,
                               geometry_msgs::Pose &workPose){
    pcl::PointCloud<PointT>::Ptr disk_cloud(new pcl::PointCloud<PointT>);
    _convert_to_pcl(clusterCloud, disk_cloud);
    _estimate_normals(disk_cloud);
    //displayCloud(disk_cloud);
    pcl::ModelCoefficients coeff;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // if any cylinder present
    //displayCloud(disk_cloud);
    if(!_get_cylinder(disk_cloud, minRadius, maxRadius, coeff, inliers)){
        ROS_WARN_NAMED(PRCPN_LGRNM,"Couldn't find disk in given cloud cluster!");
        return false;
    }
    _tube_cloud.reset(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setIndices(inliers);
    extract.setInputCloud(disk_cloud);
    extract.setNegative(false);
    extract.filter(*_tube_cloud);

    _axis_points.reset(new pcl::PointCloud<PointT>);
    _r = coeff.values[6];
    _collaps_normals(); //resets _raw_axis_points pointer. uses _r and _tube_cloud
    _num_of_points = _raw_axis_points->points.size();
    inliers->indices.clear();
    TubePerception::Cylinder cyl;
    pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients line_coeff;
    int is_strong;
    _find_line(_raw_axis_points, line_inliers, line_coeff, is_strong);
    pcl::PointCloud<PointT>::Ptr axis_points(new pcl::PointCloud<PointT>);
    _project_points_on_line(_raw_axis_points, line_inliers, line_coeff, axis_points);
    PointT p1, p2;
    pcl::getMaxSegment(*axis_points, p1, p2);
    if(is_strong>0){
        cyl.isStrong = true;// should
    }
    else{
        cyl.isStrong = false;
    }
    cyl.p1.setX(p1.x);
    cyl.p1.setY(p1.y);
    cyl.p1.setZ(p1.z);

    cyl.p2.setX(p2.x);
    cyl.p2.setY(p2.y);
    cyl.p2.setZ(p2.z);

    cyl.radius = _r;

    // compute pose and copy cylinder object
    //had to use following from _define_poses. behaviour of mat.setValue is not fully known
    tf::Transform iden = tf::Transform::getIdentity();
    tf::Vector3 ux, uy, uz = cyl.getAxisVector(iden);
    uz.normalize();
    ux = _get_perp_vec3(uz);
    ux.normalize();
    uy = uz.cross(ux);
    uy.normalize();
    tf::Matrix3x3 mat;
    mat.setValue(ux.getX(), uy.getX(), uz.getX(),
                 ux.getY(), uy.getY(), uz.getY(),
                 ux.getZ(), uy.getZ(), uz.getZ() );
    tf::Vector3 mid_point = cyl.getMidPoint(iden);

    tf::Transform disk_tf(mat, mid_point);
    disk.setPose(tf2pose(disk_tf));
    disk.p1 = cyl.p1;
    disk.p2 = cyl.p2;
    disk.isStrong = cyl.isStrong;
    disk.radius = cyl.radius;

    //make x axis horizontal
    tf::Transform xform0;
    xform0.setIdentity();
    xform0.setOrigin(tf::Vector3(1,0,0));

    xform0 = disk_tf * xform0;
    tf::Vector3 x_axis_of_disk = xform0.getOrigin() - disk_tf.getOrigin();
    //project x axis of disk on x-z plan of global frame
    x_axis_of_disk.setY(0.0);
    //angle between world x_axis and x_axis of disk in x-z plan
    double angle = x_axis_of_disk.angle(tf::Vector3(1,0,0));
    xform0.setIdentity();

    tf::Vector3 axis = cyl.getAxisVector(iden);
    if(axis.getY()>0){
        if(x_axis_of_disk.getZ()>0)
            xform0.setRotation(tf::Quaternion(tf::Vector3(0,0,1),angle));
        else
            xform0.setRotation(tf::Quaternion(tf::Vector3(0,0,-1),angle));
    }
    else{
        if(x_axis_of_disk.getZ()>0)
            xform0.setRotation(tf::Quaternion(tf::Vector3(0,0,-1),angle));
        else
            xform0.setRotation(tf::Quaternion(tf::Vector3(0,0,1),angle));
    }
    tf::Transform xform;
    xform.setIdentity();
    if(axis.getY()>0)
        xform.setRotation(tf::Quaternion(tf::Vector3(1,0,0),M_PI/2));
    else
        xform.setRotation(tf::Quaternion(tf::Vector3(-1,0,0),M_PI/2));
    xform.setOrigin(tf::Vector3(-disk.radius, 0, 0));
    tf::Transform work_pose = disk_tf * xform0 * xform;
    workPose = tf2pose(work_pose);
    return true;
}

//! \brief
void CloudProcessing::_convert_to_pcl(const sensor_msgs::PointCloud2 &rosTubeCloud, pcl::PointCloud<PointT>::Ptr pcl_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(rosTubeCloud,*cloud);
    pcl_cloud->header = cloud->header;
    pcl_cloud->height = cloud->height;
    pcl_cloud->width = cloud->width;
    pcl_cloud->is_dense = cloud->is_dense==1;
    //pcl_cloud->sensor_orientation_ = cloud->sensor_orientation_;
    //pcl_cloud->sensor_origin_ = Eigen::Vector4f (0.0, 0.0, 1.5, 0.0f);

    pcl_cloud->points.resize(cloud->points.size());
    for(unsigned int i=0; i<cloud->points.size(); i++){
        pcl_cloud->points[i].x = cloud->points[i].x;
        pcl_cloud->points[i].y = cloud->points[i].y;
        pcl_cloud->points[i].z = cloud->points[i].z;
        pcl_cloud->points[i].rgb = cloud->points[i].rgb;
    }
}

/*void CloudProcessing::_convert_to_ros(pcl::PointCloud<PointT>::Ptr pcl_cloud, const sensor_msgs::PointCloud2 &ros_cloud)
{
    pcl::toROSMsg(*pcl_cloud, ros_cloud);
}*/

void CloudProcessing::segmentizeCloud(const sensor_msgs::PointCloud2 &cloudIn)
{
    pcl::PointCloud<PointT>::Ptr hull_points(new pcl::PointCloud<PointT>);
    sensor_msgs::PointCloud2 ros_cloud;
    if(cloudIn.header.frame_id.compare("/base_link")==0 || cloudIn.header.frame_id.compare("base_link"))
        _convert_cloud_to("/base_link", cloudIn, ros_cloud);
    else
        ros_cloud = cloudIn;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    _convert_to_pcl(ros_cloud, cloud);
    _segmentize_cloud(cloud, hull_points);
}

bool CloudProcessing::_convert_cloud_to(std::string target_frame, const sensor_msgs::PointCloud2 &cloud_in, sensor_msgs::PointCloud2 &cloud_out)
{
    //std::string target_frame = "/base_link";
    if(cloud_in.header.frame_id.compare("base_link")==0 || cloud_in.header.frame_id.compare("/base_link")==0){
        cloud_out = cloud_in;
        return true;
    }
    tf::TransformListener listener;
    tf::StampedTransform stamped_tf;
    tf::Transform tf;

    try{
        listener.waitForTransform(target_frame.c_str(), cloud_in.header.frame_id.c_str(), ros::Time(0), ros::Duration(3) );
        listener.lookupTransform(target_frame.c_str(), cloud_in.header.frame_id.c_str(),ros::Time(0), stamped_tf);
    }
    catch(tf::TransformException ex){
        ROS_ERROR_NAMED(PRCPN_LGRNM,"%s",ex.what());
        ROS_ERROR_NAMED(PRCPN_LGRNM,"Failed to convert point cloud from %s frame to %s frame", target_frame.c_str(), cloud_in.header.frame_id.c_str());
        return false;
    }
    tf.setOrigin(stamped_tf.getOrigin());
    tf.setRotation(stamped_tf.getRotation());
    pcl_ros::transformPointCloud(target_frame, tf, cloud_in, cloud_out);
    return true;
}

void CloudProcessing::_segmentize_cloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr hull_points)
{
    //straight from example
    //double z_min = 0., z_max = 0.05; // we want the points above the plane, no farther than 5 cm from the surface
    //pcl::PointCloud<PointT>::Ptr hull_points(new pcl::PointCloud<PointT>);
    pcl::ConvexHull<PointT> hull;
    hull.setDimension (2); // not necessarily needed, but we need to check the dimensionality of the output
    hull.setInputCloud (cloud);
    hull.reconstruct (*hull_points);

   /* pcl::PointCloud<PointT>::Ptr point_cloud(new pcl::PointCloud<PointT>);
    pcl::PointIndices cloud_indices;
    if (hull.getDimension () == 2)
    {
      pcl::ExtractPolygonalPrismData<PointT> prism;
      prism.setInputCloud (*point_cloud);
      prism.setInputPlanarHull (*hull_points);
      prism.setHeightLimits (z_min, z_max);
      prism.segment (cloud_indices);
    }
    else
     ROS_ERROR_NAMED(PRCPN_LGRNM,"The input cloud does not represent a planar surface.\n");

    _estimate_normals(cloud);*/

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (hull_points,"tube_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tube_cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->spin();
}

//random 45/90degree circular trajectory. in global frame
//now in local frame
void CloudProcessing::_generate_work_vectors(unsigned int cyl_idx, tf::Vector3 at_point)
{
    /*double l = (double)rand()/(double)RAND_MAX; //Not working. Always generates 0.840188
    l = 0.6;
    
    int cyl_idx = rand()%(_tube->cylinders.size()+1);
    cyl_idx = 0;*/

    //tf::Transform tube_tf = _tube->getTransform();
    tf::Transform tube_tf = tf::Transform::getIdentity();
    tf::Vector3 axis = _tube->cylinders[cyl_idx].getAxisVector(tube_tf); //in local
    axis.normalize();

    // point on axis of cylinder
    /*tf::Vector3 at_point = _tube->cylinders[cyl_idx].getAxisVector(tube_tf);
    at_point *= l;
    at_point += _tube->cylinders[cyl_idx].p1; //convert from cylinder to tube frame*/

    tf::Vector3 perp_vec = _get_perp_vec3(axis);
    tf::Vector3 point, vec1, vec2;

    vec1 = perp_vec.rotate(axis, ((double)rand()/(double)RAND_MAX)*M_PI);
    //vec1 = perp_vec;
    double angle = 0;
    for(unsigned int i=0; i<4; i++){
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        PointT pointnormal;
        for(unsigned int j=2; j<=45; j+=2){
            angle = (((double)i*M_PI)/4) + (((double)j*M_PI)/180);
            //std::cout<<"\n"<<angle;
            vec2 = vec1.rotate(axis,angle);
            vec2.normalize();
            point = at_point + (vec2*_tube->cylinders[cyl_idx].radius);
            pointnormal.x = point.x();
            pointnormal.y = point.y();
            pointnormal.z = point.z();
            pointnormal.normal_x = vec2.x();
            pointnormal.normal_y = vec2.y();
            pointnormal.normal_z = vec2.z();
            cloud->points.push_back(pointnormal);
        }
        _tube->workPointsCluster.push_back(cloud);
        //vec1 = vec1.rotate(axis,(M_PI/2));
        //std::cout<<"\n";
    }
}

void CloudProcessing::_generate_work_vectors(pcl::PointCloud<PointT>::Ptr interest_points)
{
    if(interest_points->points.empty()){
        ROS_ERROR_NAMED(PRCPN_LGRNM,"No intereset point!");
        return;
    }

    PointT test_point_pcl = interest_points->points.at(0);

    tf::Transform tube_tf = _tube->getTransform(), test_point_tf;
    test_point_tf.setIdentity();
    test_point_tf.setOrigin(tf::Vector3(test_point_pcl.x, test_point_pcl.y, test_point_pcl.z));
    tf::Transform local_point_tf;
    local_point_tf = tube_tf.inverseTimes(test_point_tf);
    tf::Vector3 origin_local_point = local_point_tf.getOrigin();
    test_point_pcl.x = origin_local_point.getX();
    test_point_pcl.y = origin_local_point.getY();
    test_point_pcl.z = origin_local_point.getZ();

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    coeff->header.frame_id = "/base_link";
    coeff->header.stamp = ros::Time::now();
    coeff->values.resize(6);
    int cyl_idx = _tube->whichCylinder(test_point_pcl);
    tf::Vector3 p1, p2;
    pcl::PointCloud<PointT>::Ptr points_out(new pcl::PointCloud<PointT>);
    if(cyl_idx<_tube->cylinders.size()){
        p1 = _tube->cylinders[cyl_idx].getGlobalP1(tube_tf);
        p2 = _tube->cylinders[cyl_idx].getGlobalP2(tube_tf);
        coeff->values[0] = p1.getX();
        coeff->values[1] = p1.getY();
        coeff->values[2] = p1.getZ();
        coeff->values[3] = p2.getX()-p1.getX();
        coeff->values[4] = p2.getY()-p1.getY();
        coeff->values[5] = p2.getZ()-p1.getZ();

        pcl::ProjectInliers<PointT> proj;
        proj.setModelType (pcl::SACMODEL_LINE);
        proj.setInputCloud (interest_points);
        proj.setModelCoefficients (coeff);
        proj.filter(*points_out);
    }

    if(points_out->points.empty()){
        ROS_ERROR_NAMED(PRCPN_LGRNM,"No interest point has been projected on line");
        return;
    }

    PointT avg_point;
    avg_point.x = avg_point.y = avg_point.z = 0.0;
    for(unsigned int i=0; i<points_out->points.size(); i++){
        avg_point.x += points_out->points[i].x;
        avg_point.y += points_out->points[i].y;
        avg_point.z += points_out->points[i].z;
    }
    avg_point.x /= points_out->points.size();
    avg_point.y /= points_out->points.size();
    avg_point.z /= points_out->points.size();

    tf::Transform avg_point_tf;
    avg_point_tf.setIdentity();
    avg_point_tf.setOrigin(tf::Vector3(avg_point.x, avg_point.y, avg_point.z));
    avg_point_tf = tube_tf.inverseTimes(avg_point_tf);


    tf::Vector3 at_point;
    at_point = avg_point_tf.getOrigin();
    _generate_work_vectors(cyl_idx, at_point);
}

bool CloudProcessing::writePointCloudOnfile(const sensor_msgs::PointCloud2 &rosCloud, std::string fileName){
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>);
    _convert_to_pcl(rosCloud, pcl_cloud);
    if(pcl::io::savePCDFileASCII(fileName,*pcl_cloud)==-1)
        return false;
    return true;
}

bool CloudProcessing::writePointCloudOnfile(const pcl::PointCloud<PointT>::Ptr pclCloud,
                           std::string fileName)
{
    if(pcl::io::savePCDFileASCII(fileName,*pclCloud)==-1)
        return false;
    return true;
}

bool CloudProcessing::writeAxisPointsOnFile(std::string fileName){
    if(pcl::io::savePCDFileASCII(fileName,*_axis_points)==-1)
        return false;
    return true;
}

bool CloudProcessing::readPointCloud(std::string fileName, pcl::PointCloud<PointT>::Ptr cloudOut)
{
    if(pcl::io::loadPCDFile(fileName,*cloudOut)==-1)
        return false;
    return true;
}

tf::Vector3 CloudProcessing::_get_perp_vec3(tf::Vector3 v3){
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
//Assumes that all cylinders are in global frame at this point
//Z is axis vector
void CloudProcessing::_define_pose(void)
{
    tf::Transform identity_tf = tf::Transform::getIdentity();
    // compute cylinder's global pose
    for(size_t i=0; i<_tube->cylinders.size(); i++)
    {
        geometry_msgs::Pose pose;
        tf::Vector3 mid_point = _tube->cylinders[i].getMidPoint(identity_tf);
        pose.position.x = mid_point.getX();
        pose.position.y = mid_point.getY();
        pose.position.z = mid_point.getZ();
        //pose.position.x = (_tube->cylinders[i].p1.x + _tube->cylinders[i].p2.x) / 2;
        //pose.position.y = (_tube->cylinders[i].p1.y + _tube->cylinders[i].p2.y) / 2;
        //pose.position.z = (_tube->cylinders[i].p1.z + _tube->cylinders[i].p2.z) / 2;

        tf::Vector3 ux, uy, uz = _tube->cylinders[i].getAxisVector(identity_tf);
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
        //For temporary use only. Will get converted in local to tube frame
        _tube->cylinders[i].setPose(pose);
    }

    if(!_tube->cylinders.empty()){
        int idx = 0;
        geometry_msgs::Pose pose;
        pose = _tube->cylinders[idx].getPose();
        _tube->setPose(pose);  //Tube's pose = first (assuming to be strong) cylinder's global pose
    }

    // Convert cylinder from global to local to tube
    for(size_t i=0; i<_tube->cylinders.size(); i++)
    {
        tf::Transform tube_tf, cyl_tf, tube_cyl_tf;
        cyl_tf = _tube->cylinders[i].getTransform();
        tube_tf = _tube->getTransform();
        tube_cyl_tf = tube_tf.inverseTimes(cyl_tf);
        _tube->cylinders[i].setPose(tube_cyl_tf);
        tf::Transform p1_tf, p2_tf;

        // convert points in to local frame
        p1_tf.setIdentity();
        p2_tf.setIdentity();
        p1_tf.setOrigin(_tube->cylinders[i].p1);
        p2_tf.setOrigin(_tube->cylinders[i].p2 );

        p1_tf = tube_tf.inverseTimes(p1_tf);
        p2_tf = tube_tf.inverseTimes(p2_tf);

        _tube->cylinders[i].p1 = p1_tf.getOrigin();
        _tube->cylinders[i].p2 = p2_tf.getOrigin();
    }
}

void CloudProcessing::_compensate_error(void){
    for(size_t i=0; i<_tube_cloud->points.size(); i++)
        _tube_cloud->points[i].z -= _z_error;
}

void CloudProcessing::_estimate_normals(pcl::PointCloud<PointT>::Ptr cloud){
    pcl::NormalEstimation<PointT, PointT> ne;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);

    ne.setRadiusSearch(0.01);
    ROS_INFO_NAMED(PRCPN_LGRNM, "Using fixed view point x=0.0 y=0.0 z=1.5");
    ne.setViewPoint(0.0,0.0,1.5);
    ne.compute(*cloud);
}

double CloudProcessing::_get_radius(){
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
    seg.setRadiusLimits (_r_min, _r_max);
    seg.setProbability(0.99);
    seg.setInputCloud (_tube_cloud);
    seg.setInputNormals (_tube_cloud);
    seg.setDistanceFromOrigin(0.05);
    // Obtain the cylinder inliers and coefficients
    seg.segment (inliers, coeff);
    double r = coeff.values.at(6);
    if(inliers.indices.size()<_tube_cloud->points.size()*_weak_line_thr){
        r=0;
    }
    ROS_INFO_NAMED(PRCPN_LGRNM, "Found cylinder radius using cylinder RANSAC: %f", coeff.values.at(6));
    return  r;//_r.push_back(coeff.values[6]);
}

bool CloudProcessing::_get_cylinder(pcl::PointCloud<PointT>::Ptr cloud,
                                    double r_min, double r_max,
                                    pcl::ModelCoefficients &coeff,
                                    pcl::PointIndices::Ptr inliers)
{
    pcl::SACSegmentationFromNormals<PointT, PointT> seg0;
    pcl::ModelCoefficients coeff0;
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    seg0.setModelType(pcl::SACMODEL_PLANE);
    seg0.setMethodType(pcl::SAC_RANSAC);
    seg0.setNormalDistanceWeight(0.1);
    seg0.setMaxIterations(10000);
    seg0.setDistanceThreshold(0.001);
    seg0.setProbability(0.8);
    seg0.setInputCloud(cloud);
    seg0.setInputNormals(cloud);
    seg0.segment(*plane_inliers,coeff0);
    //displayCloud(cloud);
    //std::cout<<"\nplane inliers : "<<plane_inliers->indices.size()<<'\n';
    _remove_inliers(cloud,plane_inliers);
    plane_inliers->indices.clear();
    //displayCloud(cloud);
    seg0.segment(*plane_inliers,coeff0);
    //std::cout<<"\nplane inliers : "<<plane_inliers->indices.size()<<'\n';
    _remove_inliers(cloud,plane_inliers);
    //displayCloud(cloud);

    pcl::SACSegmentationFromNormals<PointT, PointT> seg;
    //pcl::ModelCoefficients coeff;
    //pcl::PointIndices inliers;
    coeff.values.clear();
    // Create the segmentation object for cylinder segmentation and set all the parameters
    //seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (100000);
    seg.setDistanceThreshold (0.001);
    seg.setRadiusLimits (r_min, r_max);
    seg.setProbability(0.8);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud);
    seg.setDistanceFromOrigin(0.5);
    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers, coeff);
    if(inliers->indices.empty()){
        ROS_WARN_NAMED(PRCPN_LGRNM,"No cylinder found in given cloud");
        return false;
    }
    if(inliers->indices.size()<(cloud->points.size()/100)){ // if inliers are less than 1%
        ROS_WARN_NAMED(PRCPN_LGRNM,
                       "Cylinder found but (%d) number of inliers are not enough. (%d total number of points)",
                       inliers->indices.size(), cloud->points.size());
        return false;
    }
    if(coeff.values[6]<r_min || coeff.values[6]>r_max){
         ROS_WARN_NAMED(PRCPN_LGRNM,"No cylinder for [%f, %f] radius range",r_min, r_max);
         for(int i=0; i<coeff.values.size(); i++){
             coeff.values[i] = 0.0;
         }
         return false;
    }
    else{
        ROS_INFO_NAMED(PRCPN_LGRNM, "Cylinder of %f radius found using RANSAC", coeff.values[6]);
    }
    return true;
}

// resets _raw_axis_point pointer
// needs _tube_cloud initialized and _r
void CloudProcessing::_collaps_normals(void)
{
    _raw_axis_points.reset(new pcl::PointCloud<PointT>);
    _raw_axis_points->header = _tube_cloud->header;
    _raw_axis_points->points.resize(_tube_cloud->points.size());

    for(unsigned int i=0; i<_tube_cloud->points.size(); i++){
        _raw_axis_points->points[i].x = _tube_cloud->points[i].x - _tube_cloud->points[i].normal_x * _r;
        _raw_axis_points->points[i].y = _tube_cloud->points[i].y - _tube_cloud->points[i].normal_y * _r;
        _raw_axis_points->points[i].z = _tube_cloud->points[i].z - _tube_cloud->points[i].normal_z * _r;
    }
    _raw_axis_points->width = _raw_axis_points->points.size();
    _raw_axis_points->height = 1;
    //get_largest_cluster(raw_axis_points, axis_points);
}


// gets segments of line and stores them as cylinder object in given tube
void CloudProcessing::_segmentize_axis(void)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    Cylinder cyl;
    PointT p1, p2;
    int is_strong;
    pcl::ModelCoefficients coeff;
    char loop_nr = '0';
    std::string fname;
    while(_find_line(_raw_axis_points, inliers, coeff, is_strong)){
        loop_nr++;
        //********************************
        pcl::PointCloud<PointT>::Ptr raw_axis_inliers(new pcl::PointCloud<PointT>);
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(_raw_axis_points);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*raw_axis_inliers);

        fname = "raw_axis_inliers_";
        fname += loop_nr;
        std::cout<<'\n'<<fname<<'\n';
        displayCloud(raw_axis_inliers,fname);
        //********************************
        pcl::PointCloud<PointT>::Ptr axis_points(new pcl::PointCloud<PointT>);
        _project_points_on_line(_raw_axis_points, inliers, coeff, axis_points);
        pcl::getMaxSegment(*axis_points, p1, p2);
        fname = "cyl_axis_inliers_";
        fname += loop_nr;
        std::cout<<'\n'<<fname<<'\n';
        displayCloud(axis_points, fname);
        // set cylinder variables
        // haven't defined poses yet
        if(is_strong>0){
            cyl.isStrong = true;
        }
        else{
            cyl.isStrong = false;
        }
        cyl.p1.setX(p1.x);
        cyl.p1.setY(p1.y);
        cyl.p1.setZ(p1.z);

        cyl.p2.setX(p2.x);
        cyl.p2.setY(p2.y);
        cyl.p2.setZ(p2.z);

        cyl.radius = _r;

        // add axis points in to _axis_points cloud which is collection of all axis points
        for(unsigned int i=0; i<axis_points->points.size(); i++){
            _axis_points->points.push_back(axis_points->points[i]);
        }
        // add cylinder in to tube
        _tube->cylinders.push_back(cyl);

        // remove current inliers from _raw_axis_points;
        _remove_inliers(_raw_axis_points,inliers);
        fname = "raw_axis_";
        fname += loop_nr;
        std::cout<<'\n'<<fname<<'\n';
        displayCloud(_raw_axis_points,fname);
        // reuse of indices
        inliers->indices.clear();

        // extract all points resting inside cylinder.
        _cylinder_filter(p1,p2,_r, _raw_axis_points, inliers);

        axis_points->points.clear();
        // project these points who weren't part of line inliers but are still inside cylinder
        // and store them in _axis_points cloud so we do not loos information if we need to use rgb processing on axis_points
        _project_points_on_line(_raw_axis_points, inliers, coeff, axis_points);
        for(unsigned int i=0; i<axis_points->points.size(); i++){
            _axis_points->points.push_back(axis_points->points[i]);
        }
        fname = "axis_";
        fname += loop_nr;
        std::cout<<'\n'<<fname<<'\n';
        displayCloud(_axis_points,fname);
        // remove those points from _raw_axis_points if already hasn't been removed as a line inliers
        _remove_inliers(_raw_axis_points,inliers);
        fname = "raw_axis_2_";
        fname += loop_nr;
        std::cout<<'\n'<<fname<<'\n';
        displayCloud(_raw_axis_points,fname);
        // and get ready for next run
        inliers->indices.clear();
    }
    _axis_points->header = _tube_cloud->header;
    _axis_points->width = _axis_points->points.size();
    _axis_points->height = 1;
    ROS_INFO_NAMED(PRCPN_LGRNM,"%d axis points found",_axis_points->points.size());
    ROS_INFO_NAMED(PRCPN_LGRNM,"Tube model generated with %d cylinder(s)", _tube->cylinders.size());
}


// runs Line RANSAC on to _raw_axis_points cloud and fills up inliers
// if number of inliers meets threshold return true, false otherwise
bool CloudProcessing::_find_line(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients &coeff, int &is_strong)
{
    if(_raw_axis_points->points.size()< (_min_points*_num_of_points) ) //check if there are enough points to generate model
        return false;
    pcl::SACSegmentation<PointT> seg;
    //pcl::ModelCoefficients coeff;
    //pcl::PointIndices inliers;

    // Create the segmentation object for line and set all the parameters
    // get inliers and coefficients with harder parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setOptimizeCoefficients(true);
    seg.setProbability(0.99);
    seg.setMaxIterations (20000);
    seg.setDistanceThreshold (0.001);
    seg.setInputCloud (cloud_in);

    // Obtain the Line inliers and coefficients
    seg.segment (*inliers, coeff);

    is_strong = -1;
    ROS_INFO_NAMED(PRCPN_LGRNM,"%d line inliers",inliers->indices.size());
    // if there are enough points to go forward with
    if(inliers->indices.size()>=(_strong_line_thr*_num_of_points)){
        is_strong = 1;
        ROS_INFO_NAMED(PRCPN_LGRNM,"Passes strong line threshold");
        return true;
    }
    else{
        ROS_INFO_NAMED(PRCPN_LGRNM,"No enough points. Re-running Line RANSAC.");
        seg.setProbability(0.96);
        seg.setMaxIterations (20000);
        seg.setDistanceThreshold (0.003);
        inliers->indices.clear();
        seg.segment (*inliers, coeff);
        // if there is still any valid line exists in _raw_axis_points set found_line but it would be weak
        if(inliers->indices.size()>=_weak_line_thr){
            return true;
        }
    }
    return false;
}

// extracts inliers from _raw_axis_points
// projects inliers on to line defined by line_coeff
void CloudProcessing::_project_points_on_line(pcl::PointCloud<PointT>::Ptr cloud_in,
                                              pcl::PointIndices::Ptr inliers,
                                              pcl::ModelCoefficients line_coeff,
                                              pcl::PointCloud<PointT>::Ptr points_out){
    pcl::ExtractIndices<PointT> extract;
    pcl::PointCloud<PointT>::Ptr inlier_points(new pcl::PointCloud<PointT>);
    // extract inlier points (part of line definition) from _raw_axis_points
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*inlier_points);

    // make copy of coefficients into new pointer object to use that in setModelCoefficient function
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    coeff->header = line_coeff.header;
    coeff->values.resize(line_coeff.values.size()); //size should be 6
    for(unsigned int i=0; i<line_coeff.values.size(); i++)
        coeff->values[i] = line_coeff.values[i];

    // project line in_ points (part of line definition) on to axis defined in coeff
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_LINE);
    proj.setInputCloud (inlier_points);
    proj.setModelCoefficients (coeff);
    proj.filter(*points_out);
}

void CloudProcessing::_remove_inliers(pcl::PointCloud<PointT>::Ptr points,  std::vector<int> &indices){
    pcl::PointIndices::Ptr pcl_indices(new pcl::PointIndices);
    pcl_indices->indices.resize(indices.size());
    for(size_t i=0; i<indices.size(); i++)
        pcl_indices->indices[i] = indices[i];
    _remove_inliers(points,pcl_indices);
}

void CloudProcessing::_remove_inliers(pcl::PointCloud<PointT>::Ptr points, pcl::PointIndices::Ptr indices){
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(points);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*points);
}

// returns indices of points resting inside of given cylinder pareameters
void CloudProcessing::_cylinder_filter(PointT p1, PointT p2, double radius,
                                       pcl::PointCloud<PointT>::Ptr cloud_in,
                                       pcl::PointIndices::Ptr inliers){
    int pts_cnt = 0;
    tf::Vector3 p1_vec, p2_vec;
    p1_vec.setX(p1.x);
    p1_vec.setY(p1.y);
    p1_vec.setZ(p1.z);

    p2_vec.setX(p2.x);
    p2_vec.setY(p2.y);
    p2_vec.setZ(p2.z);

    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;

    float l_sq = (dx*dx) + (dy*dy) + (dz*dz);
    float r_sq = radius*radius;

    tf::Vector3 test_point;
    for(size_t i=0; i<cloud_in->points.size(); i++){
        test_point.setValue(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
        if(isInCylinder(p1_vec, p2_vec, l_sq, r_sq, test_point)){
            inliers->indices.push_back(i);
            pts_cnt++;
        }
    }
    ROS_INFO_NAMED(PRCPN_LGRNM,"%d points found within cylinder (filter)",pts_cnt);
}

void CloudProcessing::setZerror(float error){
    _z_error = error;
}

void CloudProcessing::displayCloudWithNormals(pcl::PointCloud<PointT>::Ptr cloud, std::string fname, char r, char g, char b)
{
    /*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.27,0.27,0.27);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud, r, g, b);
    viewer->addPointCloud<PointT> (cloud,single_color,"cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer->addPointCloudNormals<PointT, PointT> (cloud, cloud, 10, 0.02, "normals");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.8, 0.8, "normals");
    //viewer->setCameraPose/*(0.65,-0.2,0.7,0,0,0,0.85,0.0,0.5);
    //(0.00304872,3.04872
    // 0.553262,-0.212465,0.682817
    // -0.129723,-0.498943,0.868818
    // 0.164084,0.231407,0.95892
    // 0.523599
    //1615,1026
    //65,24);
    viewer->camera_.clip[0] = 0.00304872;
    viewer->camera_.clip[1] = 3.04872;

    viewer->camera_.focal[0] = 0.553262;
    viewer->camera_.focal[1] = -0.212465;
    viewer->camera_.focal[2] = 0.682817;

    viewer->camera_.pos[0] = -0.129723;
    viewer->camera_.pos[1] = -0.498943;
    viewer->camera_.pos[2] = 0.868818;

    viewer->camera_.view[0] = 0.164084;
    viewer->camera_.view[1] = 0.231407;
    viewer->camera_.view[2] = 0.95892;

    viewer->camera_.fovy = 0.5236;

    viewer->camera_.window_size[0] = 1615;
    viewer->camera_.window_size[1] = 1026;

    viewer->camera_.window_pos[0] = 65;
    viewer->camera_.window_pos[1] = 24;
    viewer->updateCamera();
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    //viewer->setFullScreen(true);
    //viewer->saveScreenshot(fname);
    viewer->spin();*/
}

void CloudProcessing::displayCloud(pcl::PointCloud<PointT>::Ptr cloud,std::string fname, char r, char g, char b)
{
    /*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.27,0.27,0.27);
    //viewer->setBackgroundColor (0,0,0);
    //pcl::visualization::PointCloudColorHandlerRGBField<PointT> color(cloud);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud,r,g,b);
    //viewer->addPointCloud<PointT> (cloud,color,"cloud");
    viewer->addPointCloud<PointT> (cloud, single_color,"cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    //(0.00304872,3.04872
    // 0.553262,-0.212465,0.682817
    // -0.129723,-0.498943,0.868818
    // 0.164084,0.231407,0.95892
    // 0.523599
    //1615,1026
    //65,24);
    viewer->camera_.clip[0] = 0.00304872;
    viewer->camera_.clip[1] = 3.04872;

    viewer->camera_.focal[0] = 0.553262;
    viewer->camera_.focal[1] = -0.212465;
    viewer->camera_.focal[2] = 0.682817;

    viewer->camera_.pos[0] = -0.129723;
    viewer->camera_.pos[1] = -0.498943;
    viewer->camera_.pos[2] = 0.868818;

    viewer->camera_.view[0] = 0.164084;
    viewer->camera_.view[1] = 0.231407;
    viewer->camera_.view[2] = 0.95892;

    viewer->camera_.fovy = 0.523599;

    viewer->camera_.window_size[0] = 1615;
    viewer->camera_.window_size[1] = 1026;

    viewer->camera_.window_pos[0] = 65;
    viewer->camera_.window_pos[1] = 24;
    viewer->updateCamera();
    //viewer->setFullScreen(true);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    //viewer->setFullScreen(true);
    //viewer->saveScreenshot(fname);
    //viewer->spinOnce(0.1);
    viewer->spin();*/
}

void CloudProcessing::displayCloud2(pcl::PointCloud<PointT>::Ptr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (1,1,1);
    //viewer->setBackgroundColor (0,0,0);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> color(cloud);
    viewer->addPointCloud<PointT> (cloud,color,"cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");
    //viewer->addPointCloudNormals<PointT, PointT> (cloud, cloud, 5, 0.02, "normals");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.33, 0.33, 0.33, "normals");
    //(0.00304872,3.04872
    // 0.553262,-0.212465,0.682817
    // -0.129723,-0.498943,0.868818
    // 0.164084,0.231407,0.95892
    // 0.523599
    //1615,1026
    //65,24);
    viewer->camera_.clip[0] = 0.00304872;
    viewer->camera_.clip[1] = 3.04872;

    viewer->camera_.focal[0] = 0.553262;
    viewer->camera_.focal[1] = -0.212465;
    viewer->camera_.focal[2] = 0.682817;

    viewer->camera_.pos[0] = -0.129723;
    viewer->camera_.pos[1] = -0.498943;
    viewer->camera_.pos[2] = 0.868818;

    viewer->camera_.view[0] = 0.164084;
    viewer->camera_.view[1] = 0.231407;
    viewer->camera_.view[2] = 0.95892;

    viewer->camera_.fovy = 0.523599;

    viewer->camera_.window_size[0] = 1615;
    viewer->camera_.window_size[1] = 1026;

    viewer->camera_.window_pos[0] = 65;
    viewer->camera_.window_pos[1] = 24;
    viewer->updateCamera();
    //viewer->setFullScreen(true);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    //viewer->setFullScreen(true);
    //viewer->saveScreenshot(fname);
    //viewer->spinOnce(0.1);
    viewer->spin();
}

}//TubePerception

