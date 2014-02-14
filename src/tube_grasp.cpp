#include "tubeGrasp.h"
#include "math.h"


namespace TubeGrasp
{

Grasp::Grasp()
{
}

GraspAnalysis::GraspAnalysis(TubePerception::Tube::Ptr tube, ros::NodeHandle nh)
{
    //grasp_array_ = grasp_array;
    tube_ = tube;
    grasp_array_.reset(new (TubeGrasp::GraspArray));
    test_pairs_.reset(new (TubeGrasp::GraspPairArray));
    valid_pairs_.reset(new (TubeGrasp::GraspPairArray));
    axis_step_size_ = 0.05;
    circular_steps_ = 8;
    wrist_axis_offset_ = 0.072; //72 mm from axis of cylinder to wrist origin
    nodeHandle = nh;
    MAX_TEST_GRASPS = 30;
    MAX_ITERATION = 200;
}

void GraspAnalysis::setWorkPose(geometry_msgs::Pose &p)
{
    work_pose_ = p;
}

int GraspAnalysis::getWorkPose(geometry_msgs::Pose &p)
{
    if( !(work_pose_.orientation.x == 0 &&
        work_pose_.orientation.y == 0 &&
        work_pose_.orientation.z == 0 &&
        work_pose_.orientation.w == 1 &&
        work_pose_.position.x == 0 &&
        work_pose_.position.y == 0 &&
        work_pose_.position.z == 0) )
    {
        p = work_pose_;
        return 1;
    }
    return 0;
}

void GraspAnalysis::setWorkTrajIdx(int trajIdx)
{
    traj_idx_ = trajIdx;
}

void GraspAnalysis::analyze()
{
    gen_work_trajectory_();
    gen_grasps_(axis_step_size_, circular_steps_, grasp_array_);
    gen_test_pairs_();
    //test_pairs_for_ik_();
    //compute_metric_();
}

void GraspAnalysis::pickUpTube()
{
    geometry_msgs::Pose pick_pose, aprh_pose;
    pick_pose = getPickUpPose();
    
    tf::Transform p, a;
    a.setIdentity();
    a.setOrigin(tf::Vector3(-0.01, 0, 0));
    p = pose2tf(pick_pose);
    a = p*a;
    aprh_pose = tf2pose(a);
    
    Gripper r_grpr("right_arm");
    r_grpr.open();
    dualArms da(nodeHandle);
    da.moveRightArm(aprh_pose);
    //r_grpr.open();
    //da.moveRightArm(pick_pose);
    //r_grpr.setPosition(tube_->cylinders[0].radius*1.7,100);
}

geometry_msgs::Pose GraspAnalysis::getPickUpPose()
{
    //will generate only vertical grasps
    GraspArray::Ptr grasp_array;
    grasp_array.reset(new (GraspArray));
    gen_grasps_(axis_step_size_, 1, grasp_array);
    
    //tf::Vector3 ref_point;
    geometry_msgs::Point ref_point;
    ref_point.x = 0;
    ref_point.y = 0;
    ref_point.z = 0;
    geometry_msgs::Pose pose;
    for(size_t i=0; i<tube_->cylinders.size(); i++)
    {
        pose = tube_->getCylinderGlobalPose(i);
        ref_point.x += pose.position.x;
        ref_point.y += pose.position.y;
        ref_point.z += pose.position.z;
    }
    ref_point.x /= tube_->cylinders.size();
    ref_point.y /= tube_->cylinders.size();
    ref_point.z /= tube_->cylinders.size();

    std::vector<double> dist(grasp_array->grasps.size());
    tf::Vector3 c,r(ref_point.x, ref_point.y, ref_point.z);
    double max = 1000;
    unsigned int grasp_idx = 0;
    for(size_t i=0; i<grasp_array->grasps.size(); i++)
    {
        c.setValue(grasp_array->grasps[i].wristPose.position.x,
                   grasp_array->grasps[i].wristPose.position.y,
                   grasp_array->grasps[i].wristPose.position.z );
        dist[i] = c.distance(r);
        if(dist[i] < max)
        {
            grasp_idx = i;
            max = dist[i];
        }
    }
    geometry_msgs::Pose grasp_pose = grasp_array->grasps[grasp_idx].wristPose;
    return grasp_pose;
}

void GraspAnalysis::gen_grasps_(double axis_step_size, int circular_steps, GraspArray::Ptr grasp_array)
{
    TubeGrasp::Grasp grasp;

    tf::Quaternion quaternion;
    tf::Transform step_tf,wrist_axis_tf, tf_grasp_cyl, tf_grasp_tube;

    //wrist_axis_tf.setOrigin(tf::Vector3(0.0, wrist_axis_offset_,0.0));
    //quaternion.setEulerZYX(-(M_PI/2), 0.0, (M_PI/2));
    wrist_axis_tf.setOrigin(tf::Vector3(wrist_axis_offset_,0.0,0.0));
    quaternion.setEulerZYX(M_PI, 0, 0);
    wrist_axis_tf.setRotation(quaternion); //if offset is in Y then -90,0,90

    // Circular group number. Grasps are devided in groups so that it reduces number of total pairs
    unsigned int group_n = 0;

    for(size_t i=0; i<tube_->cylinders.size(); i++)
    {
        //floor value
        tf::Vector3 axis_vec = tube_->cylinders[i].getAxisVector();
        float axis_len = axis_vec.length();
        int axis_steps = axis_len/axis_step_size;

        for(int j=1; j<=axis_steps; j++)
        {
            group_n++;
            // if X is Cylinder Axis
            //step_tf.setOrigin( tf::Vector3( (j*axis_step_size), 0.0, 0.0 ) );
            // if Z is Cylinder Axis
            step_tf.setOrigin(tf::Vector3( 0, 0, ((j*axis_step_size)-(axis_len/2)) ) );
            float circular_step_size = 2*M_PI/circular_steps;
            for(int k=0; k<circular_steps; k++)
            {
                quaternion.setEulerZYX(k*circular_step_size, 0, 0);
                step_tf.setRotation(quaternion);
                tf_grasp_cyl = step_tf*wrist_axis_tf;
                tf_grasp_tube = tube_->cylinders[i].getLocalTransform() * tf_grasp_cyl;  //temp local transform is not working
                //tf_grasp_tube = tube_->cylinders[i].getGlobalTransform() * tf_grasp_cyl;
                tf::Vector3 orig = tf_grasp_tube.getOrigin();
                tf::Quaternion q = tf_grasp_tube.getRotation();
                grasp.wristPose.position.x = orig.x();
                grasp.wristPose.position.y = orig.y();
                grasp.wristPose.position.z = orig.z();
                grasp.wristPose.orientation.x = q.x();
                grasp.wristPose.orientation.y = q.y();
                grasp.wristPose.orientation.z = q.z();
                grasp.wristPose.orientation.w = q.w();

                grasp.group = group_n;
                grasp.cylinderIdx = i;

                grasp_array->grasps.push_back(grasp);
            }
        }
    }
    ROS_INFO("%d grasps generated",grasp_array->grasps.size());
}

void GraspAnalysis::normalize_worktrajectory_()
{

    /*geometry_msgs::Pose prev_pose,crnt_pose;
    float theta_dist;
    tf::Vector3 axis_dist;

    for(size_t i=0; i<wotk_traj_.poses.size(); i++)
    {
        if(i==0)
            prev_pose=wotk_traj_.poses[0];

        crnt_pose = wotk_traj_.poses[i];
        tf::Quaternion crnt_q, prev_q;

        crnt_q.setValue(crnt_pose.orientation.x,
                        crnt_pose.orientation.y,
                        crnt_pose.orientation.z,
                        crnt_pose.orientation.w );

        prev_q.setValue(prev_pose.orientation.x,
                        prev_pose.orientation.y,
                        prev_pose.orientation.z,
                        prev_pose.orientation.w );
        theta_dist = crnt_q.getAngle() - prev_q.getAngle();
        axis_dist  = crnt_q.getAxis() - prev_q.getAxis();
        //ROS_INFO_STREAM("Angle: "<<theta_dist<<"  Axis: "<<axis_dist);
    }*/
}

void GraspAnalysis::xform_in_tubeframe_()
{

    geometry_msgs::Pose p;
    tf::Transform point, tube, xform;
    tube = tube_->getTransform();
    for(size_t i=0; i<work_traj_.poses.size(); i++)
    {
        p = work_traj_.poses[i];
        point = pose2tf(p);
        xform = tube.inverseTimes(point);
        work_traj_.poses[i] = tf2pose(xform);
    }
}

bool GraspAnalysis::gen_work_trajectory_()
{
    pcl::PointCloud<PointT>::Ptr cloud;
    tf::Vector3 cyl_axis,ux,uz,uy;  // right hand rule -> x, z, y
    geometry_msgs::Pose pose;
    work_traj_.header.frame_id = "base_link";
    work_traj_.header.stamp= ros::Time::now();

    //***visualization purpose only***
    vismsg_workNormalsX.header.frame_id = "base_link";
    vismsg_workNormalsX.header.stamp = ros::Time::now();
    vismsg_workNormalsX.ns = "WorkNormalsX";
    vismsg_workNormalsX.action = visualization_msgs::Marker::ADD;
    vismsg_workNormalsX.id = 1;
    vismsg_workNormalsX.type = visualization_msgs::Marker::LINE_LIST;
    vismsg_workNormalsX.scale.x = 0.001;
    vismsg_workNormalsX.color.r = 1.0;
    vismsg_workNormalsX.color.a = 1.0;
    //***Visualization purpose only***

    //***visualization purpose only***
    vismsg_workNormalsY.header.frame_id = "base_link";
    vismsg_workNormalsY.header.stamp = ros::Time::now();
    vismsg_workNormalsY.ns = "WorkNormalsY";
    vismsg_workNormalsY.action = visualization_msgs::Marker::ADD;
    vismsg_workNormalsY.id = 1;
    vismsg_workNormalsY.type = visualization_msgs::Marker::LINE_LIST;
    vismsg_workNormalsY.scale.x = 0.001;
    vismsg_workNormalsY.color.g = 1.0;
    vismsg_workNormalsY.color.a = 1.0;
    //***Visualization purpose only***

    //***visualization purpose only***
    vismsg_workNormalsZ.header.frame_id = "base_link";
    vismsg_workNormalsZ.header.stamp = ros::Time::now();
    vismsg_workNormalsZ.ns = "WorkNormalsZ";
    vismsg_workNormalsZ.action = visualization_msgs::Marker::ADD;
    vismsg_workNormalsZ.id = 1;
    vismsg_workNormalsZ.type = visualization_msgs::Marker::LINE_LIST;
    vismsg_workNormalsZ.scale.x = 0.001;
    vismsg_workNormalsZ.color.b = 1.0;
    vismsg_workNormalsZ.color.a = 1.0;
    //***Visualization purpose only***

    for(size_t i=0; i<tube_->workPointsCluster.size(); i++)
    {
        cloud = tube_->workPointsCluster[i];

        for(size_t j=0; j<cloud->points.size(); j++)
        {
            PointT point;
            point = cloud->points[j];
            unsigned int cyl_idx = tube_->whichCylinder(point);
            if(cyl_idx != tube_->cylinders.size())
                cyl_axis =tube_->cylinders[cyl_idx].getAxisVector();
            else
            {
                ROS_ERROR("GraspAnalysis - Couldn't get cylinder index");
                return false;
            }
            cyl_axis.normalize();
            ux.setValue(point.normal_x, point.normal_y, point.normal_z);
            ux.normalize();
            uy = ux.cross(cyl_axis);
            uy.normalize();
            uz = ux.cross(uy);
            uz.normalize();


            //***Visualization purpose only***
            geometry_msgs::Point p;
            p.x = point.x; p.y = point.y; p.z = point.z;
            vismsg_workNormalsX.points.push_back(p);
            p.x = point.x + (ux.x()*0.05);
            p.y = point.y + (ux.y()*0.05);
            p.z = point.z + (ux.z()*0.05);
            vismsg_workNormalsX.points.push_back(p);

            p.x = point.x; p.y = point.y; p.z = point.z;
            vismsg_workNormalsY.points.push_back(p);
            p.x = point.x + (uy.x()*0.05);
            p.y = point.y + (uy.y()*0.05);
            p.z = point.z + (uy.z()*0.05);
            vismsg_workNormalsY.points.push_back(p);

            p.x = point.x; p.y = point.y; p.z = point.z;
            vismsg_workNormalsZ.points.push_back(p);
            p.x = point.x + (uz.x()*0.05);
            p.y = point.y + (uz.y()*0.05);
            p.z = point.z + (uz.z()*0.05);
            vismsg_workNormalsZ.points.push_back(p);
            //***Visualization purpose only***

            tf::Matrix3x3 mat;

            mat.setValue(ux.getX(), uy.getX(), uz.getX(),
                         ux.getY(), uy.getY(), uz.getY(),
                         ux.getZ(), uy.getZ(), uz.getZ());

            /*mat.setValue(ux.getX(), ux.getY(), ux.getZ(),
                           uy.getX(), uy.getY(), uy.getZ(),
                           uz.getX(), uz.getY(), uz.getZ());*/

            tf::Quaternion q,q2,q3;
            mat.getRotation(q);
            tf::Vector3 vec(1, 0, 0);
            q2.setRotation(vec, (M_PI/2));
            q3 = q*q2;
            //Rotate +/- 90 degrees around X to align Y to Axis.
            //Weird but only work around as of now.
            pose.orientation.x = q3.getX();
            pose.orientation.y = q3.getY();
            pose.orientation.z = q3.getZ();
            pose.orientation.w = q3.getW();
            pose.position.x = point.x;
            pose.position.y = point.y;
            pose.position.z = point.z;
            work_traj_.poses.push_back(pose);
        }
    }
    //normalize_worktrajectory_();
    //xform_in_tubeframe_();
    work2tube_trajectory_();
    return true;
}

void GraspAnalysis::work2tube_trajectory_()
{
    tf::Transform W, t, p;
    geometry_msgs::Pose pose;
    if(getWorkPose(pose))
        W = pose2tf(pose);
    else
    {
        ROS_ERROR("GraspAnalysis - WorkPose is not set yet.");
        return;
    }
    tube_traj_.header.frame_id = "base_link";
    tube_traj_.header.stamp = ros::Time::now();
    tube_traj_.poses.resize(work_traj_.poses.size());
    for(size_t i=0; i<work_traj_.poses.size(); i++)
    {
        p = pose2tf(work_traj_.poses[i]);
        p = p.inverse();
        t = W*p;
        tube_traj_.poses[i] = tf2pose(t);
    }
}


void GraspAnalysis::gen_test_pairs_()
{
    TubeGrasp::GraspPair gp;
    for(size_t i=0; i<grasp_array_->grasps.size(); i++)
    {
        for(size_t j=0; j<grasp_array_->grasps.size(); j++)
        {
            if(i!=j &&
               grasp_array_->grasps[i].group!=grasp_array_->grasps[j].group)
            {
                gp.rightGrasp = grasp_array_->grasps[i];
                gp.leftGrasp = grasp_array_->grasps[j];
                test_pairs_->graspPairs.push_back(gp);
            }
        }
    }
    ROS_INFO_STREAM("Total Grasp Pairs : "<<test_pairs_->graspPairs.size());
}

void GraspAnalysis::test_pairs_for_ik_()
{
    dualArms da(nodeHandle);

    int idx;
    unsigned long test_grasps=1;

    ROS_INFO_STREAM("Checking "<<MAX_ITERATION<<" randomly selected grasps for ik...");
    GraspPair gp;
    da.objPoseTraj = tube_traj_;

    unsigned long it=MAX_ITERATION;
    valid_pairs_->graspPairs.reserve(MAX_TEST_GRASPS);
    while(it>0)
    {
        std::cout<<'.';
        it--;
        idx = rand()%(test_pairs_->graspPairs.size()+1);
        gp = test_pairs_->graspPairs[idx];
        da.rightWristOffset = pose2tf(gp.rightGrasp.wristPose);
        da.leftWristOffset = pose2tf(gp.leftGrasp.wristPose);
        //genTrajectory clears vectors(qRight, qLeft) in arguement
        if(da.genTrajectory(gp.qRight, gp.qLeft))
        {
            gp.isValid = true;
            valid_pairs_->graspPairs.push_back(gp);
            test_grasps++;
            if(test_grasps>MAX_TEST_GRASPS)
                break;
        }
        else
        {
            std::cout<<" ";
            //fail_cnt++;
        }
    }
    std::cout<<"\n";
    if(!valid_pairs_->graspPairs.empty())
    {
        ROS_INFO_STREAM("GraspAnalysis - "<<valid_pairs_->graspPairs.size()
                        <<" valid pairs found from "
                        <<MAX_ITERATION<<" iteration");
    }
    else
        ROS_WARN_STREAM("No pair found to be valid for IK");
}


//To compute manipulability metric and assign rank
//for each trajectory point and ultimatlly accumulative rank
void GraspAnalysis::compute_metric_()
{
    ManipAnalysis ma_right("right_arm",nodeHandle);
    ManipAnalysis ma_left("left_arm",nodeHandle);
    tf::Vector3 f_vec,axis,vec;
    tf::Transform t, work = pose2tf(work_pose_);
    t.setIdentity();
    t.setOrigin(tf::Vector3(1,0,0));
    t = work * t;
    vec = t.getOrigin();
    f_vec = work.getOrigin() - vec;
    f_vec.normalize();

    t.setIdentity();
    t.setOrigin(tf::Vector3(0,1,0));
    t = work * t;
    vec = t.getOrigin();
    axis = work.getOrigin() - vec;
    axis.normalize();

    ma_right.setReferencePoint(work.getOrigin());
    ma_left.setReferencePoint(work.getOrigin());
    ma_right.setRotationAxis(axis);
    ma_left.setRotationAxis(axis);
    ma_right.setForceVec(f_vec);
    ma_left.setForceVec(f_vec);

    unsigned int nr_of_joints;
    TubeGrasp::GraspPair gp;
    std::vector<double> q_r, q_l;
    double rfm, lfm, rrm, lrm; //force metric and rotation metric

    //For each valid pair
    for(size_t i=0; i<valid_pairs_->graspPairs.size(); i++)
    {
        gp = valid_pairs_->graspPairs[i];
        nr_of_joints = gp.qRight.size()/tube_traj_.poses.size();
        //resize to total number of trajectory points
        valid_pairs_->graspPairs[i].forceMetric.resize(tube_traj_.poses.size());
        valid_pairs_->graspPairs[i].rotMetric.resize(tube_traj_.poses.size());

        //For each pair, for each traj point, compute and store metrics
        double f_min=1000, r_min=1000;
        for(size_t j=0; j<tube_traj_.poses.size(); j++)
        {
            q_r.clear();
            q_r.resize(nr_of_joints);
            q_l.clear();
            q_l.resize(nr_of_joints);
            //Extract 7 joint values from linear qRight/qLeft vector.
            for(size_t k=0; k<nr_of_joints; k++)
            {
                q_r[k] = gp.qRight[(j*nr_of_joints)+k];
                q_l[k] = gp.qLeft[(j*nr_of_joints)+k];
            }
            rfm = ma_right.getForceMetric(q_r);
            rrm = ma_right.getRotationMetric(q_r);
            lfm = ma_left.getForceMetric(q_l);
            lrm = ma_left.getRotationMetric(q_l);

            valid_pairs_->graspPairs[i].forceMetric[j] = std::min(rfm,lfm);
            if(std::min(rfm,lfm)<f_min)
                f_min = std::min(rfm,lfm);
            valid_pairs_->graspPairs[i].rotMetric[j] = std::min(rrm,lrm);
            if(std::min(rrm,lrm)<r_min)
                r_min = std::min(rrm,lrm);
        }
        valid_pairs_->graspPairs[i].minForce = f_min;
        valid_pairs_->graspPairs[i].minRot = r_min;
        valid_pairs_->graspPairs[i].rank = (f_min*0.5) + (r_min*0.5);
        ROS_INFO_STREAM("Pair "<<i<<" (F,R): "<<f_min<<" "<<r_min);
    }
    double r = 0;
    unsigned int best_grasp;
    for(size_t i=0; i<valid_pairs_->graspPairs.size(); i++)
    {
        if(valid_pairs_->graspPairs[i].rank>r)
        {
            best_grasp = i;
            r = valid_pairs_->graspPairs[i].rank;
        }
    }
    ROS_WARN_STREAM("Best Grasp Pair index: "<<best_grasp);
    /*dualArms da(nodeHandle);
    da.objPoseTraj = tube_traj_;
    da.rightWristOffset = pose2tf(valid_pairs_->graspPairs[best_grasp].rightGrasp.wristPose);
    da.leftWristOffset = pose2tf(valid_pairs_->graspPairs[best_grasp].leftGrasp.wristPose);
    while(getchar()!='q');
    da.executeJointTrajectory(valid_pairs_->graspPairs[best_grasp].qRight,
                              valid_pairs_->graspPairs[best_grasp].qRight);*/
}

void GraspAnalysis::getGraspMarker(visualization_msgs::MarkerArray &markerArray)
{
    visualization_msgs::Marker marker, x_axis, z_axis;

    markerArray.markers.clear();

    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "GraspOrigins";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.id = 1;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.005;

    geometry_msgs::Pose pose;
    TubeGrasp::Grasp g;
    tf::Transform wrist;
    for(size_t i=0; i<grasp_array_->grasps.size(); i++)
    {
        g = grasp_array_->grasps[i];
        wrist = pose2tf(g.wristPose);
        wrist = tube_->getTransform()*wrist;
        pose = tf2pose(wrist);
        marker.pose = pose;
        markerArray.markers.push_back(marker);
        marker.id++;
    }

    x_axis.header.frame_id = "base_link";
    x_axis.header.stamp = ros::Time::now();
    x_axis.ns = "Approach";
    x_axis.action = visualization_msgs::Marker::ADD;
    x_axis.id = 1;
    x_axis.type = visualization_msgs::Marker::LINE_LIST;
    x_axis.scale.x = 0.001;
    x_axis.color.r = 1.0;
    x_axis.color.a = 1.0;

    geometry_msgs::Point p;
    tf::Transform step,g_tf, r_tf;
    tf::Vector3 vec;
    for(size_t i=0; i<grasp_array_->grasps.size(); i++)
    {
        g = grasp_array_->grasps[i];
        wrist = pose2tf(g.wristPose);
        wrist = tube_->getTransform()*wrist;
        pose = tf2pose(wrist);
        p.x = pose.position.x;
        p.y = pose.position.y;
        p.z = pose.position.z;
        x_axis.points.push_back(p);

        step.setOrigin(tf::Vector3(0.02,0,0));
        g_tf.setOrigin(tf::Vector3(pose.position.x,
                                   pose.position.y,
                                   pose.position.z ));
        g_tf.setRotation(tf::Quaternion(pose.orientation.x,
                                        pose.orientation.y,
                                        pose.orientation.z,
                                        pose.orientation.w ));
        r_tf = g_tf*step;
        vec = r_tf.getOrigin();
        p.x = vec.x();
        p.y = vec.y();
        p.z = vec.z();
        x_axis.points.push_back(p);
    }

    markerArray.markers.push_back(x_axis);

    z_axis.header.frame_id = "base_link";
    z_axis.header.stamp = ros::Time::now();
    z_axis.ns = "Orientation";
    z_axis.action = visualization_msgs::Marker::ADD;
    z_axis.id = 1;
    z_axis.type = visualization_msgs::Marker::LINE_LIST;
    z_axis.scale.x = 0.001;
    z_axis.color.b = 1.0;
    z_axis.color.a = 1.0;
    for(size_t i=0; i<grasp_array_->grasps.size(); i++)
    {
        g = grasp_array_->grasps[i];
        wrist = pose2tf(g.wristPose);
        wrist = tube_->getTransform()*wrist;
        pose = tf2pose(wrist);
        p.x = pose.position.x;
        p.y = pose.position.y;
        p.z = pose.position.z;
        z_axis.points.push_back(p);

        step.setOrigin(tf::Vector3(0,0,0.01));
        g_tf.setOrigin(tf::Vector3(pose.position.x,
                                   pose.position.y,
                                   pose.position.z ));
        g_tf.setRotation(tf::Quaternion(pose.orientation.x,
                                        pose.orientation.y,
                                        pose.orientation.z,
                                        pose.orientation.w ));
        r_tf = g_tf*step;
        vec = r_tf.getOrigin();
        p.x = vec.x();
        p.y = vec.y();
        p.z = vec.z();
        z_axis.points.push_back(p);
    }
    markerArray.markers.push_back(z_axis);
}
}// NAMESPACE TUBEGRASP
