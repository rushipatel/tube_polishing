#include <ros/ros.h>
#include "controlSequence.h"
#include "kdl_ik.h"

#define SEGMENTATION_SRV "/tabletop_segmentation"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "tube_polishing");
    ros::NodeHandlePtr nh(new ros::NodeHandle);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ControlSequence control_seq(nh);
    control_seq.initialize();
    control_seq.start();
    ros::shutdown();
}
