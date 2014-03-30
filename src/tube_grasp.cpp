#include "tubeGrasp.h"
#include "math.h"


namespace TubeGrasp
{

double Grasp::getWristOffset(void){
    return _wrist_offset;
}

geometry_msgs::Pose Grasp::getWristGlobalPose(const geometry_msgs::Pose &objectPose)
{
    tf::Transform obj = pose2tf(objectPose), grasp = getWristTransform();
    grasp = obj*grasp;
    return tf2pose(grasp);
}

tf::Transform Grasp::getWristGlobalPose(const tf::Transform &objectTf)
{
    tf::Transform grasp = getWristTransform();
    grasp = objectTf * grasp;
    return grasp;
}

// Do not use if not necessory
geometry_msgs::Pose Grasp::getPose(void){
    return _pose;
}

tf::Transform Grasp::getTransform(void){
    return pose2tf(_pose);
}

geometry_msgs::Pose Grasp::getWristPose(){
    ROS_WARN_COND(_wrist_offset==0.0, "Wrist Offset is set to zero. Did you forget to update it using setWristOffset()?");
    return tf2pose(_get_wrist_pose(_wrist_offset));
}

tf::Transform Grasp::getWristTransform(){
    ROS_WARN_COND(_wrist_offset==0.0, "Wrist Offset is set to zero. Did you forget to update it using setWristOffset()?");
    return _get_wrist_pose(_wrist_offset);
}

tf::Transform Grasp::_get_wrist_pose(double offset){
    tf::Transform xform;
    xform.setIdentity();
    xform.setOrigin(tf::Vector3(-offset, 0.0, 0.0));
    return (pose2tf(_pose)*xform);
}

tf::Transform Grasp::getWristTransform(double offset){
    return _get_wrist_pose(offset);
}

geometry_msgs::Pose Grasp::getWristPose(double offset){
    return tf2pose(_get_wrist_pose(offset));
}

void Grasp::setWristOffset(const double offset){
    _wrist_offset = offset;
}

void Grasp::setPose(const geometry_msgs::Pose &pose){
    _pose = pose;
}

void Grasp::setPose(const tf::Transform &tf){
    _pose = tf2pose(tf);
}

GraspAnalysis::GraspAnalysis(ros::NodeHandlePtr nh)
{
    _nh = nh;
    //_grasp_array = grasp_array;
    //_tube = tube;
    _grasp_array.reset(new (TubeGrasp::GraspArray));
    _test_pairs.reset(new (TubeGrasp::GraspPairArray));
    _valid_pairs.reset(new (TubeGrasp::GraspPairArray));
    _axis_step_size = 0.05;
    _circular_steps = 8;
    _wrist_axis_offset = 0.17; //72 mm from axis of cylinder to wrist origin
    _grasp_pair_found = false;
    _traj_idx = 0;
    MAX_TEST_GRASPS = 30;
    MAX_ITERATION = 800;
}

void GraspAnalysis::setTubePtr(TubePerception::Tube::Ptr tube){
    _tube = tube;
}

void GraspAnalysis::setWorkPose(geometry_msgs::Pose &p)
{
    _work_pose = p;
}

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

void GraspAnalysis::getTubeWorkTrajectory(geometry_msgs::PoseArray &tube_traj)
{
    tube_traj = _tube_traj;
}

void GraspAnalysis::setWorkTrajIdx(int trajIdx)
{
    _traj_idx = trajIdx;
}

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

bool GraspAnalysis::getComputedGraspPair(GraspPair &graspPair)
{
    if(_grasp_pair_found){
        graspPair = _computed_pair;
        return true;
    }
    else{
        ROS_ERROR("GraspAnalysis - There is no valid grasp pair or grasps hasn't been computed yet. did you call analyze()?");
        return false;
    }
}

//returns global pick pose
//using current_pair this function tries to avoid generating pick poses near to pointsToAvoid
TubeGrasp::Grasp GraspAnalysis::getPickPose(std::vector<tf::Vector3> &pointsToAvoid, double min_dist)
{
    //will generate only vertical grasps
    GraspArray::Ptr grasp_array;
    grasp_array.reset(new (GraspArray));
    //offset is 0.185 for picking so that fingers don't hit table.
    _gen_grasps(_axis_step_size, 180, grasp_array, 0.185);

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
        if(grasp_array->grasps[i].group==prev_grp){

            TubeGrasp::Grasp grasp = grasp_array->grasps[i];
            // compute x vector of grasp pose
            double offset = grasp.getWristOffset();
            grasp.setWristOffset(0.01); //0.01 so it doesn't complain
            tf::Transform step0 = grasp.getWristGlobalPose(t);
            grasp.setWristOffset(1.0); //remember that next function interpolates in negative x
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
        else
        {
            max_angle = M_PI*2;
            double dist;
            tf::Transform grasp_tf = grasp_array->grasps[idx].getWristGlobalPose(t);
            tf::Vector3 pick_grasp_pos = grasp_tf.getOrigin();
//            pick_grasp_pos.setValue(grasp_array->grasps[idx].wristPose.position.x,
//                                    grasp_array->grasps[idx].wristPose.position.y,
//                                    grasp_array->grasps[idx].wristPose.position.z);
            bool violeted_dist = false;
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

    //get very rough estimation of CG (reference point) to figure out nearest grasp
    //tf::Vector3 ref_point;
    geometry_msgs::Point ref_point;
    ref_point.x = 0;
    ref_point.y = 0;
    ref_point.z = 0;
    geometry_msgs::Pose pose;
    for(size_t i=0; i<_tube->cylinders.size(); i++)
    {
        pose = _tube->getCylinderGlobalPose(i);
        ref_point.x += pose.position.x;
        ref_point.y += pose.position.y;
        ref_point.z += pose.position.z;
    }
    ref_point.x /= _tube->cylinders.size();
    ref_point.y /= _tube->cylinders.size();
    ref_point.z /= _tube->cylinders.size();

    //check all sorted grasps for distance between ref point and it's origin. select closest
    std::vector<double> dist(grasp_sorted->grasps.size());
    tf::Vector3 test_point,ref(ref_point.x, ref_point.y, ref_point.z);
    double max = std::numeric_limits<double>::max(); //lets start with maximum
    unsigned int grasp_idx = 0;
    //grasp_pose_array.poses.clear();

    tf::Transform grsp_tf;
    for(size_t i=0; i<grasp_sorted->grasps.size(); i++){
        //grasp_pose_array.poses.push_back(grasp_sorted->grasps[i].wristPose);
        grsp_tf = grasp_sorted->grasps[i].getWristGlobalPose(t);
        test_point = grsp_tf.getOrigin();/*.setValue(grasp_sorted->grasps[i].wristPose.position.x,
                   grasp_sorted->grasps[i].wristPose.position.y,
                   grasp_sorted->grasps[i].wristPose.position.z );*/
        dist[i] = test_point.distance(ref); //store distance from reference point
        if(dist[i] < max){ //remember index of the grasp closest to reference point
            grasp_idx = i;
            max = dist[i];
        }
    }
    return grasp_sorted->grasps.at(grasp_idx);
}


//grasps are in tube frame
void GraspAnalysis::_gen_grasps(double axis_step_size, int circular_steps, GraspArray::Ptr grasp_array, double offset)
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
//                tf::Vector3 orig = tf_grasp_tube.getOrigin();
//                tf::Quaternion q = tf_grasp_tube.getRotation();
//                grasp.wristPose.position.x = orig.x();
//                grasp.wristPose.position.y = orig.y();
//                grasp.wristPose.position.z = orig.z();
//                grasp.wristPose.orientation.x = q.x();
//                grasp.wristPose.orientation.y = q.y();
//                grasp.wristPose.orientation.z = q.z();
//                grasp.wristPose.orientation.w = q.w();
                grasp.setPose(tf_grasp_tube);
                grasp.setWristOffset(offset);
                grasp.group = group_n;
                grasp.cylinderIdx = i;

                grasp_array->grasps.push_back(grasp);
            }
        }
    }
    ROS_INFO("%d grasps generated",grasp_array->grasps.size());
}

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
        //ROS_INFO_STREAM("Angle: "<<theta_dist<<"  Axis: "<<axis_dist);
    }*/
}

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

bool GraspAnalysis::_gen_work_trajectory()
{
    pcl::PointCloud<PointT>::Ptr cloud;
    tf::Vector3 cyl_axis,ux,uz,uy;  // right hand rule -> x, z, y
    geometry_msgs::Pose pose;
    _work_traj.header.frame_id = "/base_link";
    _work_traj.header.stamp= ros::Time::now();

    for(size_t i=0; i<_tube->workPointsCluster.size(); i++)
    {
        cloud = _tube->workPointsCluster[i];

        for(size_t j=0; j<cloud->points.size(); j++)
        {
            PointT point;
            point = cloud->points[j];
            unsigned int cyl_idx = _tube->whichCylinder(point);
            if(cyl_idx != _tube->cylinders.size()){
                tf::Transform tube_tf = _tube->getTransform();
                cyl_axis =_tube->cylinders[cyl_idx].getAxisVector(tube_tf);
            }
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
            //q2.setRotation(vec, (M_PI/2));
            q2.setValue(0,0,0,1);
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
            _work_traj.poses.push_back(pose);
        }
    }
    //_normalize_worktrajectory();
    //_xform_in_tubeframe();
    _work2tube_trajectory();
    return true;
}

void GraspAnalysis::_work2tube_trajectory()
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
    _tube_traj.header.frame_id = "/base_link";
    _tube_traj.header.stamp = ros::Time::now();
    _tube_traj.poses.resize(_work_traj.poses.size());
    for(size_t i=0; i<_work_traj.poses.size(); i++)
    {
        p = pose2tf(_work_traj.poses[i]);
        p = p.inverse();
        t = W*p;
        _tube_traj.poses[i] = tf2pose(t);
    }
}


void GraspAnalysis::_gen_test_pairs()
{
    TubeGrasp::GraspPair gp;
    for(size_t i=0; i<_grasp_array->grasps.size(); i++)
    {
        for(size_t j=0; j<_grasp_array->grasps.size(); j++)
        {
            if(i!=j &&
               _grasp_array->grasps[i].group!=_grasp_array->grasps[j].group)
            {
                gp.rightGrasp = _grasp_array->grasps[i];
                gp.leftGrasp = _grasp_array->grasps[j];
                _test_pairs->graspPairs.push_back(gp);
            }
        }
    }
    ROS_INFO_STREAM("Total Grasp Pairs : "<<_test_pairs->graspPairs.size());
}

void GraspAnalysis::_test_pairs_for_ik()
{
    TubeManipulation::Arms manip(_nh);

    unsigned int idx;
    unsigned long test_grasps=0;


    GraspPair gp;

    std::vector<unsigned int> indices(_test_pairs->graspPairs.size()), rand_indices;

    for(unsigned int i=0; i<_test_pairs->graspPairs.size(); i++){
        indices[i] = i;
    }
    unsigned int idx_of_indices;
    for(unsigned int i=0; i<MAX_ITERATION; i++){
        idx_of_indices = rand()%(indices.size());
        rand_indices.push_back(indices[idx_of_indices]);
        indices.erase(indices.begin() + idx_of_indices);
        //ROS_INFO("Size of indices : %d",indices.size());
    }

    //unsigned long it=MAX_ITERATION;
    _valid_pairs->graspPairs.reserve(rand_indices.size());
    std::string ss;
    TubeManipulation::CollisionCheck::Ptr collision_check(new TubeManipulation::CollisionCheck(_nh));
    collision_check->enableVisualization();
    arm_navigation_msgs::AttachedCollisionObject::Ptr att_obj_ptr(new arm_navigation_msgs::AttachedCollisionObject);

    ROS_INFO_STREAM("Checking "<<MAX_ITERATION<<" randomly selected grasps for ik...");
    ROS_INFO_STREAM("Press 'q' to interrupt computation and continue with available valid pair(s)");
    for(unsigned int i=0; i<rand_indices.size(); i++)
    {
        //it--;
        //idx = rand()%(_test_pairs->graspPairs.size()+1);
        idx = rand_indices[i];
        gp = _test_pairs->graspPairs[idx];
        geometry_msgs::Pose right_wrist_pose = gp.rightGrasp.getWristPose(), left_wrist_pose=gp.leftGrasp.getWristPose();
        _tube->getAttachedObjForBothGrasps(right_wrist_pose,att_obj_ptr);
        collision_check->setAttachedObjPtr(att_obj_ptr);
        if(manip.genTrajectory(collision_check, _tube_traj,right_wrist_pose, left_wrist_pose, gp.qRight, gp.qLeft)){
            gp.isValid = true;
            _valid_pairs->graspPairs.push_back(gp);
            test_grasps++;
            if(test_grasps>MAX_TEST_GRASPS)
                break;
        }
        else{
            //fail_cnt++;
        }
        std::cout<<"\r"<<MAX_ITERATION-i<<" -> "<<test_grasps<<std::flush;
        if(kbhit()){
            ss.clear();
            std::cin>>ss;
            if(ss.compare("q")==0){
                break;
            }
        }
        //std::cout<<"\rpres 'q' to continue  ["<<MAX_ITERATION-it<<'\t'<<test_grasps<<']'<<std::flush;
    }
    std::cout<<'\n';
    if(!_valid_pairs->graspPairs.empty()){
        ROS_INFO_STREAM("GraspAnalysis - "<<_valid_pairs->graspPairs.size()
                        <<" valid pairs found from "
                        <<MAX_ITERATION<<" iteration");
    }
    else
        ROS_WARN_STREAM("No pair found to be valid for IK");
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

            _valid_pairs->graspPairs[i].forceMetric[j] = std::min(rfm,lfm);
            if(std::min(rfm,lfm)<f_min)
                f_min = std::min(rfm,lfm);
            _valid_pairs->graspPairs[i].rotMetric[j] = std::min(rrm,lrm);
            if(std::min(rrm,lrm)<r_min)
                r_min = std::min(rrm,lrm);
        }
        _valid_pairs->graspPairs[i].minForce = f_min;
        _valid_pairs->graspPairs[i].minRot = r_min;
        _valid_pairs->graspPairs[i].rank = (f_min*0.5) + (r_min*0.5);
        ROS_INFO_STREAM("Pair "<<i<<" (F,R): "<<f_min<<" "<<r_min);
    }
    double r = 0;
    unsigned long int best_grasp;
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
        ROS_WARN_STREAM("Best Grasp Pair index: "<<best_grasp);
        _computed_pair = _valid_pairs->graspPairs[best_grasp];
        _grasp_pair_found = true;
    }
    else
        ROS_ERROR("TubeGrasp - No computed valid grasp found");


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
