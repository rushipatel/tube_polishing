#include <ros/ros.h>
#include "controlSequence.h"
#include "tubePerception.h"

#define SEGMENTATION_SRV "/tabletop_segmentation"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "tube_polishing");
    ros::NodeHandlePtr nh(new ros::NodeHandle);

    ros::Publisher cloud_pub = nh->advertise<sensor_msgs::PointCloud2>("/tube_polishing/cloud", 2);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    TubePerception::CloudProcessing cloud_process;

    ControlSequence control_seq(nh);
    control_seq.initialize();
    control_seq.start();
    ros::shutdown();
}
