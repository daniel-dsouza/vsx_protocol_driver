//
// Created by ddsouza on 6/23/23.
//

#ifndef VSX_PROTOCOL_DRIVER_ROS_VTE7500_DRIVER_H
#define VSX_PROTOCOL_DRIVER_ROS_VTE7500_DRIVER_H

#include "vsx_protocol_driver/PF.VsxProtocolDriver.WrapperNE.h"

#include "dynamic_reconfigure/server.h"
#include "vsx_protocol_driver_ros/Vte7500DriverConfig.h"

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <thread>

namespace vsx_protocol_driver_ros
{

using ReconfigureServerVte7500 = dynamic_reconfigure::Server<vsx_protocol_driver_ros::Vte7500DriverConfig>;

class Vte7500Driver : public nodelet::Nodelet
{

public:
  ~Vte7500Driver() override
  {
    task_.join();
  }

  void onInit() override
  {
    task_ = std::thread(&Vte7500Driver::run, this);
  }
private:
  std::thread task_;

  ros::Publisher cloud_pub_;

  VsxSystemHandle *vsx_system_handle_;

  std::string tf_frame_ = "sensor";

  /**
   * @{
   */
  boost::recursive_mutex reconfigure_mutex_;
  std::unique_ptr<ReconfigureServerVte7500> reconfigure_server_;
  ReconfigureServerVte7500::CallbackType f_;
  /** @} */

  template<typename T, typename F>
  T readParamFromDevice(uint32_t settings_version, std::string_view configuration_id, uint32_t configuration_version, std::string_view parameter_id, const F& func);

  void enumerateSettings(VsxSystemHandle **vsx_system_handle);

  void initializeReconfigure();

  void reconfigureCallback(Vte7500DriverConfig &config, uint32_t level);

  void run();
};

} // vsx_protocol_driver_ros

#endif //VSX_PROTOCOL_DRIVER_ROS_VTE7500_DRIVER_H
