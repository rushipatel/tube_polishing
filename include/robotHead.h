#ifndef ROBOTHEAD_H
#define ROBOTHEAD_H
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class robotHead
{
private:
  PointHeadClient* point_head_client_;

public:
  robotHead();
  ~robotHead()
  {
      delete point_head_client_;
  };
  void lookAt(double x, double y, double z);
};

#endif
