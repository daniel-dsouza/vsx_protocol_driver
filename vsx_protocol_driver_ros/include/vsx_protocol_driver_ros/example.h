//
// Created by ddsouza on 6/23/23.
//

#ifndef VSX_PROTOCOL_DRIVER_ROS_EXAMPLE_H
#define VSX_PROTOCOL_DRIVER_ROS_EXAMPLE_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <thread>

namespace vsx_protocol_driver_ros
{

class Example : public nodelet::Nodelet
{
public:
  ~Example() override
  {
    task_.join();
  }

  void onInit() override
  {
    task_ = std::thread(&Example::run, this);
  }
private:
  std::thread task_;

  ros::Publisher cloud_pub_;

  void run();
};

} // vsx_protocol_driver_ros

#endif //VSX_PROTOCOL_DRIVER_ROS_EXAMPLE_H
