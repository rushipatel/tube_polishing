#include <ros/ros.h>
//#include "controlSequence.h"

#include "manipulation.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tube_polishing");
    sleep(1);
    ros::NodeHandlePtr nh_ptr(new ros::NodeHandle);

    ros::AsyncSpinner spinner(1);
    spinner.start();


    //**********************************************

    manipulation manip(nh_ptr);
    manip.experimental();



    //**********************************************


    ROS_WARN("!!!*Work in progress*!!!");
    /*ControlSequence control_seq(rh);
    control_seq.initialize();
    control_seq.start();*/
    ros::shutdown();
}
