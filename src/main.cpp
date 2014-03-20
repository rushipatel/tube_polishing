#include <ros/ros.h>
#include "controlSequence.h"

#define SEGMENTATION_SRV "/tabletop_segmentation"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "tube_polishing");
    ros::NodeHandlePtr rh(new ros::NodeHandle);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("STARTING");
    ControlSequence control_seq(rh);
    control_seq.initialize();
    control_seq.start();
    ros::shutdown();

}
