#include <ros/ros.h>
//#include "controlSequence.h"
#include "state_machine.h"
#include "base.cpp"

#define SEGMENTATION_SRV "/tabletop_segmentation"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "tube_polishing");
    ros::NodeHandlePtr nh(new ros::NodeHandle);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    stateMachine stmc(nh);
    stmc.start();

    ros::shutdown();
}
