#include "tubeGrasp.h"

namespace TubeGrasp
{

Grasp::Grasp()
{
}

GraspAnalysis::GraspAnalysis(TubePerception::Tube::Ptr tube)
{
    //grasp_array_ = grasp_array;
    tube_ = tube;
    grasp_array_.reset(new (TubeGrasp::GraspArray));
    grasp_pairs_.reset(new (TubeGrasp::GraspPairArray));
    axis_step_size_ = 0.05;
    circular_steps_ = 8;
    wrist_axis_offset_ = 0.072; //72 mm from axis of cylinder to wrist origin
}

void GraspAnalysis::setContactVector(tf::Vector3 contactVector)
{
    contact_vector_ = contactVector;
}

void GraspAnalysis::setWorkTrajIdx(int trajIdx)
{
    traj_idx_ = trajIdx;
}

//generate grasps in global frame usually base_link
void GraspAnalysis::generate_grasps_()
{
    TubeGrasp::Grasp grasp;
    TubePerception::Cylinder cyl;

    tf::Quaternion quaternion;
    tf::Transform step_tf,wrist_axis_tf, tf_grasp_cyl, tf_grasp_tube;

    // Y or Z doesn't matter as long as X is Cylinder Axis
    wrist_axis_tf.setOrigin(tf::Vector3(0.0, wrist_axis_offset_,0.0));
    quaternion.setEulerZYX(-(M_PI/2), 0.0, (M_PI/2));
    wrist_axis_tf.setRotation(quaternion); //if offset is in Y then -90,0,90

    for(size_t i=0; i<tube_->cylinders.size(); i++)
    {
        //floor value
        int axis_steps = tube_->cylinders[i].axisVector.length()/axis_step_size_;

        for(int j=1; j<=axis_steps; j++)
        {
            // if X is Cylinder Axis
            step_tf.setOrigin( tf::Vector3( (j*axis_step_size_), 0.0, 0.0 ) );
            float circular_step_size = 2*M_PI/circular_steps_;
            for(int k=0; k<circular_steps_; k++)
            {
                quaternion.setEulerZYX(0.0, 0.0, k*circular_step_size);
                step_tf.setRotation(quaternion);
                tf_grasp_cyl = step_tf*wrist_axis_tf;
                //tf_grasp_tube_ = tube_->cylinders[i].getLocalTransform() * tf_grasp_cyl;  //temp local transform is not working
                tf_grasp_tube = tube_->cylinders[i].getGlobalTransform() * tf_grasp_cyl;
                tf::Vector3 orig = tf_grasp_tube.getOrigin();
                tf::Quaternion q = tf_grasp_tube.getRotation();
                grasp.wristPose.position.x = orig.x();
                grasp.wristPose.position.y = orig.y();
                grasp.wristPose.position.z = orig.z();
                grasp.wristPose.orientation.x = q.x();
                grasp.wristPose.orientation.y = q.y();
                grasp.wristPose.orientation.z = q.z();
                grasp.wristPose.orientation.w = q.w();

                grasp_array_->grasps.push_back(grasp);
            }
        }
    }
    ROS_INFO("%d grasps generated",grasp_array_->grasps.size());
}

bool GraspAnalysis::generate_work_trajectory_()
{
    TubePerception::Normal normal;
    TubePerception::NormalArray normal_array;
    if(traj_idx_>=tube_->workTrajectories.size())
    {
        ROS_ERROR("GraspAnalysis - given trajectory index is not valid");
        return false;
    }
    //normal_array = tube_->workTrajectories[i];
    for(size_t i=0; i<normal_array.size(); i++)
    {
        normal = normal_array[i];
        //normal.point
    }
}

void GraspAnalysis::generate_grasp_pairs_()
{
    /*TubeGrasp::GraspPair grasp_pair;
    TubeGrasp::GraspPairArray temp_pairs;
    for(size_t i=0; i<grasp_array_->grasps.size(); i++)
    {
        for(size_t j=0; j<grasp_array_->grasps.size(); j++)
        {
            if(i!=j)
            {
                grasp_pair.rightGrasp = grasp_array->grasps[i];
                grasp_pair.leftGrasp = grasp_array->grasps[j];
                temp_pairs.graspPairs.push_back(grasp_pair);
            }
        }
    }

    for(size_t i=0; i<temp_pairs.graspPairs.size(); i++)
    {
        ;
    }*/
}

//tf = current pose to machining pose
void compute_matrics( tf::Transform tf, const TubeGrasp::GraspPair &grasp_pair )
{
    grasp_pair.leftGrasp;
}

void diaplayGraspsInGlobalFrame( TubeGrasp::GraspArray::Ptr grasp_array,
                                 tf::Transform tube_tf )
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
            viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::PointXYZ p1,p2;
    viewer->setBackgroundColor (0, 0, 0);
    pcl::ModelCoefficients coeff;
    tf::Transform tf_p2inX,tf_p2inZ, tf_grasp, tf_grasp_tube,tf;
    tf::Quaternion qtn;
    tf::Vector3 orig;
    tf_p2inX.setOrigin(tf::Vector3(0.01,0,0));
    tf_p2inX.setRotation(tf::Quaternion::getIdentity());

    tf_p2inZ.setOrigin(tf::Vector3(0,0,0.005));
    tf_p2inZ.setRotation(tf::Quaternion::getIdentity());

    coeff.values.resize(4);
    coeff.values[3] = 0.002; //radius of sphere


    for (size_t i=0; i<grasp_array->grasps.size(); i++)
    {
        // X, Y, Z
                orig.setX(grasp_array->grasps[i].wristPose.position.x);
        orig.setY(grasp_array->grasps[i].wristPose.position.y);
        orig.setZ(grasp_array->grasps[i].wristPose.position.z);
        tf_grasp_tube.setOrigin(orig);
        qtn.setX(grasp_array->grasps[i].wristPose.orientation.x);
        qtn.setY(grasp_array->grasps[i].wristPose.orientation.y);
        qtn.setZ(grasp_array->grasps[i].wristPose.orientation.z);
        qtn.setW(grasp_array->grasps[i].wristPose.orientation.w);
        tf_grasp_tube.setRotation(qtn);

        tf_grasp = tube_tf * tf_grasp_tube;

        orig = tf_grasp.getOrigin();
        coeff.values[0] = orig.x();
        coeff.values[1] = orig.y();
        coeff.values[2] = orig.z();

        p1.x = orig.getX();
        p1.y = orig.getY();
        p1.z = orig.getZ();

        std::strstream ss;
        ss<<"g_"<<i;
        viewer->addSphere(coeff, ss.str());

        tf = tf_grasp*tf_p2inX;

        orig = tf.getOrigin();
        p2.x = orig.getX();
        p2.y = orig.getY();
        p2.z = orig.getZ();

        ss.flush();
        ss<<"a_"<<i;
        viewer->addLine(p1, p2, ss.str());

        tf = tf_grasp*tf_p2inZ;

        orig = tf.getOrigin();
        p2.x = orig.getX();
        p2.y = orig.getY();
        p2.z = orig.getZ();

        ss.flush();
        ss<<"o_"<<i;
        viewer->addLine(p1, p2, ss.str());
    }
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Grasps");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->spin();
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> displayGrasps(TubeGrasp::GraspArray::Ptr grasp_array)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::ModelCoefficients coeffs;

    coeffs.values.resize(4);

    coeffs.values[3] = 0.002; //radius of sphere
    for (size_t i=0; i<grasp_array->grasps.size(); i++)
    {
        // X, Y, Z
        coeffs.values[0] = grasp_array->grasps[i].wristPose.position.x;
        coeffs.values[1] = grasp_array->grasps[i].wristPose.position.y;
        coeffs.values[2] = grasp_array->grasps[i].wristPose.position.z;
        std::strstream ss;
        ss<<"grasp_"<<i;
        viewer->addSphere(coeffs, ss.str());
    }
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Grasps");
    //viewer->addCoordinateSystem (1.0);
    //viewer->initCameraParameters ();
    //viewer->spin();
    return viewer;
}

}// NAMESPACE TUBEGRASP
