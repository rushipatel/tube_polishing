#include "tubeGrasp.h"

#define GRSP_LGRNM "grsp"

namespace TubeGrasp
{

// returns set wrist offset
double Grasp::getWristOffset(void){
    return _wrist_offset;
}

// Computes pose of wrist frame in global frame given object pose
geometry_msgs::Pose Grasp::getWristGlobalPose(const geometry_msgs::Pose &objectPose)
{
    tf::Transform obj = pose2tf(objectPose), grasp = getWristTransform();
    grasp = obj*grasp;
    return tf2pose(grasp);
}

// Overloaded. takes tf type argument.
tf::Transform Grasp::getWristGlobalPose(const tf::Transform &objectTf)
{
    tf::Transform grasp = getWristTransform();
    grasp = objectTf * grasp;
    return grasp;
}

// Do not use untill absolutly necessory. raw pose.
geometry_msgs::Pose Grasp::getPose(void){
    return _pose;
}

// Same as last function. Returns pose in tf. raw pose.
tf::Transform Grasp::getTransform(void){
    return pose2tf(_pose);
}

// Returns wrist pose in local (tube) frame. Uses set offset (_wrist_offset) to compute wrist pose.
geometry_msgs::Pose Grasp::getWristPose(){
    ROS_WARN_COND_NAMED(_wrist_offset==0.0, GRSP_LGRNM, "Wrist Offset is set to zero. Did you forget to update it using setWristOffset()?");
    return tf2pose(_get_wrist_pose(_wrist_offset));
}

// Returns wrist pose in local (tube) frame. Uses set offset (_wrist_offset) to compute wrist pose.
tf::Transform Grasp::getWristTransform(){
    ROS_WARN_COND_NAMED(_wrist_offset==0.0, GRSP_LGRNM, "Wrist Offset is set to zero. Did you forget to update it using setWristOffset()?");
    return _get_wrist_pose(_wrist_offset);
}

// _pose is coinsident with cylinder axis. Computes wrist pose using given 
// offset in X axis. offset is distance from axis to wrist frame in perpendicular direction.
tf::Transform Grasp::_get_wrist_pose(double offset){
    tf::Transform xform;
    xform.setIdentity();
    xform.setOrigin(tf::Vector3(-offset, 0.0, 0.0));
    return (pose2tf(_pose)*xform);
}

// Interface function.
tf::Transform Grasp::getWristTransform(double offset){
    return _get_wrist_pose(offset);
}

// Interface fucntion
geometry_msgs::Pose Grasp::getWristPose(double offset){
    return tf2pose(_get_wrist_pose(offset));
}

// Sets offset from cylinder axis to wrist frame origin.
void Grasp::setWristOffset(const double offset){
    _wrist_offset = offset;
}

// Sets pose of grasp.
void Grasp::setPose(const geometry_msgs::Pose &pose){
    _pose = pose;
}

// overloaded.
void Grasp::setPose(const tf::Transform &tf){
    _pose = tf2pose(tf);
}

// Constructor. Sets default values and collisionObject pointer.
GraspAnalysis::GraspAnalysis(ros::NodeHandlePtr nh, collisionObjects::Ptr collisionObjs)
{
    _nh = nh;
    _collision_objects = collisionObjs;
    //_grasp_array = grasp_array;
    //_tube = tube;
    _grasp_array.reset(new (TubeGrasp::GraspArray));
    _test_pairs.reset(new (TubeGrasp::GraspPairArray));
    _valid_pairs.reset(new (TubeGrasp::GraspPairArray));
    _axis_step_size = 0.1; //in meteres
    _circular_steps = 8; //(2pi)/this number of steps
    _wrist_axis_offset = 0.17; //72 mm from axis of cylinder to wrist origin
    _grasp_pair_found = false;
    _traj_idx = 0;
    MAX_TEST_GRASPS = 30;
    MAX_ITERATION = 800;
}

// Sets pointer of Tube object. All analysis are done using tube model pointed by this pointer
void GraspAnalysis::setTubePtr(TubePerception::Tube::Ptr tube){
    _tube = tube;
}

// Work pose in global frame. Work pose is in 3D space where tube comes in to contact with machine.
void GraspAnalysis::setWorkPose(geometry_msgs::Pose &p)
{
    _work_pose = p;
}

// returns false if work pose is not set yet.
int GraspAnalysis::getWorkPose(geometry_msgs::Pose &p)
{
    if( !(_work_pose.orientation.x == 0 &&
        _work_pose.orientation.y == 0 &&
        _work_pose.orientation.z == 0 &&
        _work_pose.orientation.w == 1 &&
        _work_pose.position.x == 0 &&
        _work_pose.position.y == 0 &&
        _work_pose.position.z == 0) )
    {
        p = _work_pose;
        return 1;
    }
    return 0;
}

// 
void GraspAnalysis::getTubeWorkTrajectory(geometry_msgs::PoseArray &tube_traj)
{
    tube_traj = _tube_traj;
}

// generaly work trajectory contains multiple trajectories of tube motion. index specifies for which trajectory grasp analysis is done.
void GraspAnalysis::setWorkTrajIdx(int trajIdx)
{
    _traj_idx = trajIdx;
}

// Initiates analysis
void GraspAnalysis::compute()
{
    _grasp_pair_found = false;
    _gen_work_trajectory();
    _grasp_array->grasps.clear();
    _gen_grasps(_axis_step_size, _circular_steps, _grasp_array, _wrist_axis_offset);
    _gen_test_pairs();
    _test_pairs_for_ik();
    _compute_metric();
}

// Returns false if no computation is performed or no valid pair is found after computation.
bool GraspAnalysis::getComputedGraspPair(GraspPair &graspPair)
{
    if(_grasp_pair_found){
        graspPair = _computed_pair;
        return true;
    }
    else{
        ROS_ERROR_NAMED(GRSP_LGRNM,"There is no valid grasp pair or grasps hasn't been computed yet.");
        return false;
    }
}

//returns global pick pose
//using current_pair this function tries to avoid generating pick poses near to pointsToAvoid
TubeGrasp::Grasp GraspAnalysis::getPickPose(std::vector<tf::Vector3> &pointsToAvoid, double min_dist, geometry_msgs::PoseArray &grasp_pose_array)
{
    //will generate only vertical grasps
    GraspArray::Ptr grasp_array;
    grasp_array.reset(new (GraspArray));
    //offset is 0.185 for picking so that fingers don't hit table.
    _gen_grasps(0.025, 180, grasp_array, 0.185); // every 50 mm and 2 degree

//    tf::Transform g;
    const tf::Transform t = _tube->getTransform();

    //Convert all grasps in world frame so we can choose vertical grasps out of it
//    for(size_t i=0; i<grasp_array->grasps.size(); i++){
//        g = grasp_array->grasps[i].getPose();
//        g = t * g;
//        grasp_array->grasps[i].setPose(g);
//    }

    //store sorted(close to negative Z axis)
    GraspArray::Ptr grasp_sorted;
    grasp_sorted.reset(new (GraspArray));
    
    int prev_grp=0;
    if(!grasp_array->grasps.empty())
        prev_grp = grasp_array->grasps[0].group;
    // neg_z is to compare grasp's x axis with to check downward grasps
    tf::Vector3 neg_z(0,0,-1);//, grasp_x, vec;
//    tf::Transform x_step, grasp_step;
//    x_step.setOrigin(tf::Vector3(1,0,0));
//    x_step.setRotation(tf::Quaternion(0,0,0,1));
    int idx;
    double angle, max_angle = M_PI*2;

    for(size_t i=0; i<grasp_array->grasps.size(); i++)
    {
        //store the index of most vertical grasp within group( negative z)
        if(grasp_array->grasps[i].group==prev_grp){
            TubeGrasp::Grasp grasp = grasp_array->grasps[i];
            // compute x vector of grasp pose
            double offset = grasp.getWristOffset();
            grasp.setWristOffset(0.001); //0.001, so function doesn't complain
            tf::Transform step0 = grasp.getWristGlobalPose(t);
            grasp.setWristOffset(1.0); //remember this next function interpolates in negative x
            tf::Transform step1 = grasp.getWristGlobalPose(t);
            grasp.setWristOffset(offset); //set it back what was before

            tf::Vector3 grasp_x_vec, vec0= step0.getOrigin(), vec1 = step1.getOrigin();
            grasp_x_vec = vec0 - vec1;
//            grasp_step = grasp_array->grasps[i].getWristPose*x_step;
//            vec.setValue(grasp_array->grasps[i].wristPose.position.x,
//                         grasp_array->grasps[i].wristPose.position.y,
//                         grasp_array->grasps[i].wristPose.position.z);
//            grasp_x = grasp_step.getOrigin();
//            grasp_x = grasp_x - vec;
            angle = grasp_x_vec.angle(neg_z);
            if(angle<max_angle){
                max_angle = angle;
                idx = i;
            }
        }
        else //when group changes
        {
            max_angle = M_PI*2;
            double dist;
            tf::Transform grasp_tf = grasp_array->grasps[idx].getWristGlobalPose(t);
            tf::Vector3 pick_grasp_pos = grasp_tf.getOrigin(); //get origin of grasp pose in global frame
//            pick_grasp_pos.setValue(grasp_array->grasps[idx].wristPose.position.x,
//                                    grasp_array->grasps[idx].wristPose.position.y,
//                                    grasp_array->grasps[idx].wristPose.position.z);
            bool violeted_dist = false;
            //and compare with given vector of points if it is closer to any
            for(unsigned int i=0; i<pointsToAvoid.size(); i++){
                dist = pick_grasp_pos.distance(pointsToAvoid[i]);
                if(dist<min_dist){
                    violeted_dist = true;
                    break;
                }
            }
            if(!violeted_dist) // if pick grasp is away from two current grasps
                grasp_sorted->grasps.push_back(grasp_array->grasps[idx]);
        }
        prev_grp = grasp_array->grasps[i].group;
    }

    //get very rough estimation of center of tube (reference point) to figure out lift grasp that is near to CG
    //tf::Vector3 ref_point;
    geometry_msgs::Point ref_point;
    ref_point.x = 0;
    ref_point.y = 0;
    ref_point.z = 0;
    geometry_msgs::Pose pose;
    //length is weight
    double total_weight = 0, weight;
    for(size_t i=0; i<_tube->cylinders.size(); i++)
    {
        weight = _tube->cylinders[i].getAxisLength();
        total_weight += weight;
        pose = _tube->getCylinderGlobalPose(i);
        ref_point.x += pose.position.x * weight;
        ref_point.y += pose.position.y * weight;
        ref_point.z += pose.position.z * weight;
    }
    ref_point.x /= total_weight;
    ref_point.y /= total_weight;
    ref_point.z /= total_weight;

    //check all sorted grasps for distance between ref point and it's origin. select closest
    std::vector<double> dist(grasp_sorted->grasps.size());
    tf::Vector3 test_point,ref(ref_point.x, ref_point.y, ref_point.z);
    double min = std::numeric_limits<double>::max(); //lets start with maximum
    unsigned int grasp_idx = 0;
    grasp_pose_array.poses.clear();

    tf::Transform grsp_tf;
    Grasp test_gr;
    for(size_t i=0; i<grasp_sorted->grasps.size(); i++){
        grasp_pose_array.poses.push_back(grasp_sorted->grasps[i].getWristGlobalPose(tf2pose(t)));
        test_gr = grasp_sorted->grasps[i];
        test_gr.setWristOffset(0.0001);
        grsp_tf = test_gr.getWristGlobalPose(t);
        test_point = grsp_tf.getOrigin();/*.setValue(grasp_sorted->grasps[i].wristPose.position.x,
                   grasp_sorted->grasps[i].wristPose.position.y,
                   grasp_sorted->grasps[i].wristPose.position.z );*/
        dist[i] = test_point.distance(ref); //store distance from reference point
        //std::cout<<"\n"<<i<<"\t"<<dist[i];
        if(dist[i] < min){ //remember index of the grasp closest to reference point
            grasp_idx = i;
            min = dist[i];
        }
    }
    //std::cout<<"\n["<<grasp_idx<<"]\n";
    return grasp_sorted->grasps.at(grasp_idx);
}


//grasps are in tube frame. Generates grasps around tube. Total number of grasps 
// generated can be controlled by first two parameters. Offset is the distance 
// from axis to the origin of wrist frame.
void GraspAnalysis::_gen_grasps(double axis_step_size, int circular_steps, 
                                GraspArray::Ptr grasp_array, double offset)
{
    TubeGrasp::Grasp grasp;

    tf::Quaternion quaternion;
    tf::Transform step_tf, wrist_axis_tf, tf_grasp_cyl, tf_grasp_tube;

    //wrist_axis_tf.setOrigin(tf::Vector3(0.0, _wrist_axis_offset,0.0));
    //quaternion.setEulerZYX(-(M_PI/2), 0.0, (M_PI/2));
    wrist_axis_tf.setOrigin(tf::Vector3(offset,0.0,0.0));
    quaternion.setEulerZYX(M_PI, 0, 0);
    wrist_axis_tf.setRotation(quaternion); //if offset is in Y then -90,0,90

    // Circular group number. Grasps are devided in groups so that it reduces number of total pairs
    unsigned int group_n = 0;

    for(size_t i=0; i<_tube->cylinders.size(); i++)
    {
        //floor value
        double axis_len = _tube->cylinders[i].getAxisLength();
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
                tf_grasp_cyl = step_tf/**wrist_axis_tf*/;
                tf_grasp_tube = _tube->cylinders[i].getTransform() * tf_grasp_cyl;
                grasp.setPose(tf_grasp_tube);
                grasp.setWristOffset(offset);
                grasp.group = group_n;
                grasp.cylinderIdx = i;

                grasp_array->grasps.push_back(grasp);
            }
        }
    }
    ROS_INFO_NAMED(GRSP_LGRNM,"%d grasps generated",grasp_array->grasps.size());
}

// under construction...
void GraspAnalysis::_normalize_worktrajectory()
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
        //ROS_INFO_STREAM_NAMED(GRSP_LGRNM,"Angle: "<<theta_dist<<"  Axis: "<<axis_dist);
    }*/
}

// transforms trajectory points in to local frame.
void GraspAnalysis::_xform_in_tubeframe()
{
    geometry_msgs::Pose p;
    tf::Transform point, tube, xform;
    tube = _tube->getTransform();
    for(size_t i=0; i<_work_traj.poses.size(); i++)
    {
        p = _work_traj.poses[i];
        point = pose2tf(p);
        xform = tube.inverseTimes(point);
        _work_traj.poses[i] = tf2pose(xform);
    }
}

// given ordered points on the surface of tube, generates 6D frame. 
// Origin is the same as point. X-axis is the normal from point cloud data.
// Using normal, it accomodates surface irregularities or non cylindrical 
// (in exact sense) surface. Y is closest to cylinder axis and Z is farthest 
// from axis. With the ideal tube model and noise free, accurate point cloud 
// data, Y is the same as cylinder axis and Z is exactly perpendicular to axis.
bool GraspAnalysis::_gen_work_trajectory()
{
    pcl::PointCloud<PointT>::Ptr cloud;
    tf::Vector3 cyl_axis,ux,uz,uy;  // right hand rule -> x, z, y
    geometry_msgs::Pose pose;
    _work_traj.header.frame_id = "/base_link";
    _work_traj.header.stamp= ros::Time::now();
    _work_traj.poses.clear();
    if(_traj_idx>=_tube->workPointsCluster.size()){
        ROS_WARN_NAMED(GRSP_LGRNM,"Invalid work trajectory index");
    }

    if(_tube->workPointsCluster.empty()){
        ROS_WARN_NAMED(GRSP_LGRNM,"No work trajectory exists in given tube model");
    }
    cloud = _tube->workPointsCluster[_traj_idx];

    for(size_t j=0; j<cloud->points.size(); j++)
    {
        PointT point;
        point = cloud->points[j];
        unsigned int cyl_idx = _tube->whichCylinder(point);
        if(cyl_idx != _tube->cylinders.size()){
            tf::Transform tube_tf = _tube->getTransform();
            cyl_axis =_tube->cylinders[cyl_idx].getAxisVector(tube_tf);
        }
        else{
            ROS_ERROR_NAMED(GRSP_LGRNM,"Couldn't get cylinder index");
            return false;
        }
        cyl_axis.normalize();
        ux.setValue(point.normal_x, point.normal_y, point.normal_z);
        ux.normalize();
        uy = ux.cross(cyl_axis);
        uy.normalize();
        uz = ux.cross(uy);
        uz.normalize();

        tf::Matrix3x3 mat;

        mat.setValue(ux.getX(), uy.getX(), uz.getX(),
                     ux.getY(), uy.getY(), uz.getY(),
                     ux.getZ(), uy.getZ(), uz.getZ());

        /*mat.setValue(ux.getX(), ux.getY(), ux.getZ(),
                       uy.getX(), uy.getY(), uy.getZ(),
                       uz.getX(), uz.getY(), uz.getZ());*/

        /*double angle = ux.angle(tf::Vector3(1,0,0));
        tf::Vector3 rot_vec = ux.cross(tf::Vector3(1,0,0));
        tf::Quaternion q;
        q.setRotation(rot_vec, angle*(-1));
        tf::Transform t1(q),t2, t;
        t2.setIdentity();t2.setOrigin(tf::Vector3(0,1,0));
        t = t1*t2;
        tf::Vector3 y_of_new_frame = t.getOrigin();
        angle = uz.angle(y_of_new_frame);
        tf::Quaternion q2, q3;
        q2.setRotation(tf::Vector3(1,0,0),angle);*/

        /*tf::Quaternion q,q2,q3;
        mat.getRotation(q);
        q2.setRotation(tf::Vector3(1,0,0), (M_PI/2));
        //q2.setValue(0,0,0,1);*/
        //q3 = q*q2;
        //Rotate +/- 90 degrees around X to align Y to Axis.
        //Weird but only work around as of now.

        tf::Transform tf_norm, tf_cyl = _tube->cylinders[cyl_idx].getTransform();
        tf_cyl.setOrigin(tf::Vector3(0,0,0)); // just keep the rotation since we are working with normal(vector)
        tf_norm.setIdentity();
        tf_norm.setOrigin(tf::Vector3(point.normal_x, point.normal_y, point.normal_z));
        tf_norm = tf_cyl.inverseTimes(tf_norm); //now normal is in cylinder frame
        //ux on XY plane of local cylinder
        tf::Vector3 ux_XY = tf_norm.getOrigin();
        ux_XY.setZ(0.0);
        tf::Vector3 x_axis(1,0,0);
        //double theta_z = ux_XY.angle(tf::Vector3(1,0,0));// angle between Original X and normal in XY plane
        double theta_z = x_axis.angle(ux_XY);// angle between Original X and normal in XY plane
        //std::cout<<"\ntheta_z : "<<theta_z<<"\tux_XY.z : "<<ux_XY.z()<<"\tux_XY.y"<<ux_XY.y();
        tf::Quaternion q1, q2, q3;
        if(ux_XY.y()>0)
            q1.setRotation(tf::Vector3(0,0,1),theta_z);
        else
            q1.setRotation(tf::Vector3(0,0,-1),theta_z);
        //double theta_y = ux_XY.angle(ux);
        //q2.setRotation(tf::Vector3(0,1,0),theta_y);
        q2.setRotation(tf::Vector3(1,0,0), M_PI/2);
        q3 = q1*q2;
        tf::Transform pose_in_cyl(q3,tf::Vector3(0,0,0));
        tf_cyl = _tube->cylinders[cyl_idx].getTransform();
        tf::Transform pose_in_tube = tf_cyl * pose_in_cyl;
        pose_in_tube.setOrigin(tf::Vector3(point.x, point.y, point.z));
        pose = tf2pose(pose_in_tube);
        /*pose.orientation.x = q3.getX();
        pose.orientation.y = q3.getY();
        pose.orientation.z = q3.getZ();
        pose.orientation.w = q3.getW();
        pose.position.x = point.x;
        pose.position.y = point.y;
        pose.position.z = point.z;*/
        _work_traj.poses.push_back(pose);
    }
    //_normalize_worktrajectory();
    //_xform_in_tubeframe();
    _work2tube_trajectory(); //sets _tube_traj from work point poses
    return true;
}

// Given work trajectory and work pose, converts them in to tube trajectory.
// Assuming all work points are passing through work pose origin, what would be 
// the trajectory of tube it self.
void GraspAnalysis::_work2tube_trajectory()
{
    tf::Transform W, t, p;
    geometry_msgs::Pose pose;
    if(getWorkPose(pose))
        W = pose2tf(pose);
    else{
        ROS_ERROR_NAMED(GRSP_LGRNM,"WorkPose is not set yet.");
        return;
    }
    _tube_traj.header.frame_id = "/base_link";
    _tube_traj.header.stamp = ros::Time::now();
    _tube_traj.poses.resize(_work_traj.poses.size()+2);

    //first pose keep tube 10 mm away from wheel for the first and last trajectory point
    p = pose2tf(_work_traj.poses[0]);
    tf::Transform x_step; x_step.setIdentity();
    x_step.setOrigin(tf::Vector3(0.01,0,0));
    p *= x_step;
    p = p.inverse();
    t = W*p;
    _tube_traj.poses[0] = tf2pose(t);

    //last point
    p = pose2tf(_work_traj.poses[_work_traj.poses.size()-1]);
    p *= x_step;
    p = p.inverse();
    t = W*p;
    _tube_traj.poses[_tube_traj.poses.size()-1] = tf2pose(t);

    for(size_t i=0; i<_work_traj.poses.size(); i++){
        p = pose2tf(_work_traj.poses[i]);
        p = p.inverse();
        t = W*p;
        _tube_traj.poses[i+1] = tf2pose(t);
    }
}

// Simply, makes non-repetative pairs of sample grasps.
void GraspAnalysis::_gen_test_pairs()
{
    TubeGrasp::GraspPair gp;
    for(size_t i=0; i<_grasp_array->grasps.size(); i++){
        for(size_t j=0; j<_grasp_array->grasps.size(); j++){
            if(i!=j &&
               _grasp_array->grasps[i].group!=_grasp_array->grasps[j].group){
                gp.rightGrasp = _grasp_array->grasps[i];
                gp.leftGrasp = _grasp_array->grasps[j];
                _test_pairs->graspPairs.push_back(gp);
            }
        }
    }
    ROS_INFO_STREAM_NAMED(GRSP_LGRNM,"Total Grasp Pairs : "<<_test_pairs->graspPairs.size());
}

// 
void GraspAnalysis::_test_pairs_for_ik()
{
    unsigned int idx;
    unsigned long test_grasps=0;

    GraspPair gp;

    std::vector<unsigned int> indices(_test_pairs->graspPairs.size()), rand_indices;

    for(unsigned int i=0; i<_test_pairs->graspPairs.size(); i++){
        indices[i] = i;
    }
    unsigned int idx_of_indices;
    //get indeces of MAX_ITERATION number of non repetative random pairs
    if(MAX_ITERATION>=indices.size()){
        ROS_WARN_NAMED(GRSP_LGRNM,"MAX_ITERATION is set to %d, however total number of test pairs is %d. Updating MAX_ITERATION to %d.",
                 MAX_ITERATION, indices.size(), indices.size());
        MAX_ITERATION = indices.size();
    }

    if(MAX_TEST_GRASPS>MAX_ITERATION){
        ROS_WARN_NAMED(GRSP_LGRNM,"MAX_TEST_GRASP is set to %d, however it exceeds MAX_ITERATION. Updating MAX_TEST_GRASP to %d",MAX_TEST_GRASPS, MAX_ITERATION);
        MAX_TEST_GRASPS = MAX_ITERATION;
    }

    for(unsigned int i=0; i<MAX_ITERATION; i++){
        idx_of_indices = rand()%(indices.size());
        rand_indices.push_back(indices[idx_of_indices]);
        indices.erase(indices.begin() + idx_of_indices);
        //ROS_INFO_NAMED(GRSP_LGRNM,"Size of indices : %d",indices.size());
    }

    //unsigned long it=MAX_ITERATION;
    _valid_pairs->graspPairs.reserve(rand_indices.size());
    std::string ss;
    collisionObjects::Ptr collision_objects(new collisionObjects(_nh));
    _collision_objects->copyAllObjectsTo(collision_objects);
    arm_navigation_msgs::CollisionObject co;
    _tube->getCollisionObject(co);
    collision_objects->removeCollisionObject(co.id.c_str());
    //collision_objects->printListOfObjects();

    arm_navigation_msgs::AttachedCollisionObject::Ptr att_obj_ptr(new arm_navigation_msgs::AttachedCollisionObject);
    TubeManipulation::Arms manip(_nh, collision_objects); //arms use collision_objects
    ROS_INFO_STREAM_NAMED(GRSP_LGRNM,"Checking "<<MAX_ITERATION<<" randomly selected grasps for ik...");
    //ROS_INFO_STREAM_NAMED(GRSP_LGRNM,"Press 'q' to interrupt computation and continue with available valid pair(s)");
    ROS_INFO_STREAM_NAMED(GRSP_LGRNM,"Press return key to interrupt computation and continue with available valid pair(s)");
    for(unsigned int i=0; i<rand_indices.size(); i++)
    {
        //it--;
        //idx = rand()%(_test_pairs->graspPairs.size()+1);
        idx = rand_indices[i];
        gp = _test_pairs->graspPairs[idx];
        geometry_msgs::Pose right_wrist_pose = gp.rightGrasp.getWristPose(), left_wrist_pose=gp.leftGrasp.getWristPose();
        _tube->getAttachedObjForBothGrasps(right_wrist_pose,att_obj_ptr);
        collision_objects->addAttachedCollisionObject(*att_obj_ptr);
        //collision_objects->setPlanningScene();
        if(manip.genTrajectory(_tube_traj,right_wrist_pose, left_wrist_pose, gp.qRight, gp.qLeft)){
            gp.isValid = true;
            _valid_pairs->graspPairs.push_back(gp);
            test_grasps++;
            if(test_grasps>MAX_TEST_GRASPS)
                break;
        }
        else{
            //fail_cnt++;
        }
        std::cout<<std::flush<<"\r"<<MAX_ITERATION-i<<" -> "<<test_grasps<<std::flush;
        if(kbhit()){
            ss.clear();
            break;
        }
    }
    std::cout<<'\n';
    if(!_valid_pairs->graspPairs.empty()){
        ROS_INFO_NAMED(GRSP_LGRNM,"%d valid pairs found from %d iterations",_valid_pairs->graspPairs.size(),MAX_ITERATION);
    }
    else
        ROS_WARN_NAMED(GRSP_LGRNM,"No pair found to be valid for IK!");
}

//To compute manipulability metric and assign rank
//for each trajectory point and ultimatlly accumulative rank
void GraspAnalysis::_compute_metric()
{
    ManipAnalysis ma_right("right_arm",_nh);
    ManipAnalysis ma_left("left_arm",_nh);
    tf::Vector3 f_vec,axis,vec;
    tf::Transform t, work = pose2tf(_work_pose);
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

    f_vec.rotate(axis,M_PI/4);

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
    for(size_t i=0; i<_valid_pairs->graspPairs.size(); i++)
    {
        gp = _valid_pairs->graspPairs[i];
        nr_of_joints = gp.qRight.size()/_tube_traj.poses.size();
        //resize to total number of trajectory points
        _valid_pairs->graspPairs[i].forceMetric.resize(_tube_traj.poses.size());
        _valid_pairs->graspPairs[i].rotMetric.resize(_tube_traj.poses.size());

        //For each pair, for each traj point, compute and store metrics
        double f_min=1000, r_min=1000;
        for(size_t j=0; j<_tube_traj.poses.size(); j++)
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

            /*_valid_pairs->graspPairs[i].forceMetric[j] = std::min(rfm,lfm);
            if(std::min(rfm,lfm)<f_min) //minimum force of left and right
                f_min = std::min(rfm,lfm);*/

            _valid_pairs->graspPairs[i].forceMetric[j] = rfm + lfm;
            if((rfm+lfm)<f_min) //minimum force of left and right
                f_min = rfm + lfm;

            _valid_pairs->graspPairs[i].rotMetric[j] = std::min(rrm,lrm);
            if(std::min(rrm,lrm)<r_min)
                r_min = std::min(rrm,lrm);
        }
        _valid_pairs->graspPairs[i].minForce = f_min;
        _valid_pairs->graspPairs[i].minRot = r_min;
        _valid_pairs->graspPairs[i].rank = (f_min*0.5) + (r_min*0.5);
        ROS_INFO_STREAM_NAMED(GRSP_LGRNM,"Pair "<<i<<" (F,R): "<<f_min<<" "<<r_min);
    }
    double r = 0;
    unsigned long int best_grasp = std::numeric_limits<unsigned long int>::max();
    for(size_t i=0; i<_valid_pairs->graspPairs.size(); i++)
    {
        if(_valid_pairs->graspPairs[i].rank>r)
        {
            best_grasp = i;
            r = _valid_pairs->graspPairs[i].rank;
        }
    }
    if(!_valid_pairs->graspPairs.empty())
    {
        ROS_WARN_STREAM_NAMED(GRSP_LGRNM,"Best Grasp Pair index: "<<best_grasp);
        _computed_pair = _valid_pairs->graspPairs[best_grasp];
        _grasp_pair_found = true;
    }
    else
        ROS_ERROR_NAMED(GRSP_LGRNM,"No computed valid grasp found");


    /*dualArms da(nodeHandle);
    da.objPoseTraj = _tube_traj;
    da.rightWristOffset = pose2tf(_valid_pairs->graspPairs[best_grasp].rightGrasp.wristPose);
    da.leftWristOffset = pose2tf(_valid_pairs->graspPairs[best_grasp].leftGrasp.wristPose);
    while(getchar()!='q');
    da.executeJointTrajectory(_valid_pairs->graspPairs[best_grasp].qRight,
                              _valid_pairs->graspPairs[best_grasp].qRight);*/
}

void GraspAnalysis::getGraspMarker(TubePerception::Tube::Ptr tube, double wrist_offset, visualization_msgs::MarkerArray &markerArray)
{
    ros::Duration lifetime = ros::Duration(10);
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
    marker.color.a = 0.5;
    marker.id = 1;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.005;
    marker.lifetime = lifetime;

    //sphear at origin
    geometry_msgs::Pose pose;
    TubeGrasp::Grasp g;
    for(size_t i=0; i<_grasp_array->grasps.size(); i++)
    {
        g = _grasp_array->grasps[i];
        /*wrist = pose2tf(g.getWristGlobalPose());
        wrist = tube->getTransform()*wrist;
        pose = tf2pose(wrist);*/
        g.setWristOffset(wrist_offset);
        pose = g.getWristGlobalPose(tube->getPose());
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
    x_axis.lifetime = lifetime;

    geometry_msgs::Point p;
    tf::Transform step,g_tf, r_tf;
    tf::Vector3 vec;
    for(size_t i=0; i<_grasp_array->grasps.size(); i++)
    {
        g = _grasp_array->grasps[i];
        /*wrist = pose2tf(g.getWristGlobalPose);
        wrist = tube->getTransform()*wrist;
        pose = tf2pose(wrist);*/
        g.setWristOffset(wrist_offset);
        pose = g.getWristGlobalPose(tube->getPose());
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
    z_axis.lifetime = lifetime;
    for(size_t i=0; i<_grasp_array->grasps.size(); i++)
    {
        g = _grasp_array->grasps[i];
        /*wrist = pose2tf(g.getWristGlobalPose);
        wrist = tube->getTransform()*wrist;
        pose = tf2pose(wrist);*/
        g.setWristOffset(wrist_offset);
        pose = g.getWristGlobalPose(tube->getPose());
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
