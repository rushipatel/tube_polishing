#include <ros/ros.h>
//#include "controlSequence.h"
#include "state_machine.h"


#define SEGMENTATION_SRV "/tabletop_segmentation"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "tube_polishing");
    ros::NodeHandlePtr nh(new ros::NodeHandle);

    //ros::Publisher cloud_pub = nh->advertise<sensor_msgs::PointCloud2>("/tube_polishing/cloud", 2);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    /*ControlSequence control_seq(nh);
    control_seq.initialize();
    control_seq.start();*/

    stateMachine stmc(nh);
    stmc.start();

    /*Gripper gripper(nh);
    gripper.openRightGripper();
    gripper.openLeftGripper();
    gripper.setRightGripperPosition(0.02);
    gripper.setLeftGripperPosition(0.02);
    gripper.closeRightGripper();
    gripper.closeLeftGripper();*/
    ros::shutdown();
}
