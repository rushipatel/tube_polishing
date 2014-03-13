#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryActionGoal.h>
#include <tabletop_object_detector/Table.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometric_shapes/shapes.h>

#include <gazebo_msgs/SpawnModel.h>
#include <stdio.h>
#include <Eigen/Eigen>

#include "controlSequence.h"
#include "tubeManipulation.h"
#include "robotHead.h"
#include "tubePerception.h"
#include "tubeGrasp.h"
#include "manipAnalysis.h"
#include "gripper.h"
#include "utility.h"

#define SEGMENTATION_SRV "/tabletop_segmentation"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "tube_polishing");
    ros::NodeHandlePtr rh(new ros::NodeHandle);

    ros::AsyncSpinner spinner(1);
    spinner.start();


    ControlSequence control_seq(rh);
    control_seq.initialize();
    control_seq.start();
    ros::shutdown();

}
