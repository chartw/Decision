
#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <obstacle_detector/Obstacles.h>
#include <obstacle_detector/PangPang.h>
#include <cmath>
#include "../include/point.h"


namespace obstacle_detector
{

class pp
{
public:
  pp();
  
private:
  void pangpang(const obstacle_detector::Obstacles::ConstPtr& obstacles);

  // ROS handlers
  
  ros::NodeHandle ssibal_;
  ros::Publisher pangpang_pub_;
  ros::Subscriber pangpangsub;

  std::vector<Point> p;
};

} // namespace obstacle_detector
