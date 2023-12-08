/* 
 * Use of this source code is governed by a BSD-style 
 * license that can be found in the LICENSE file.
 */

#include "vsx_protocol_driver_ros/vte7500_driver.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


namespace vsx_protocol_driver_ros
{

template<typename T>
void vsxImageToVec(VsxImage *image, std::vector<T> &vec)
{
  const auto start = static_cast<T*>(image->rawdata);
  const size_t length = image->width * image->height;

  if (vec.capacity() < length)
  {
    vec.resize(length);
  }

  vec.assign(start, start + length);
}

template <typename T, typename F>
T Vte7500Driver::readParamFromDevice(const uint32_t settings_version, const std::string_view configuration_id, const uint32_t configuration_version, const std::string_view parameter_id, const F& func)
{
  VsxStatusCode ret = VSX_STATUS_SUCCESS;
  const char *raw_value = nullptr;

  ret = vsx_GetSingleParameterValue(vsx_system_handle_, settings_version, configuration_id.data(), configuration_version, parameter_id.data(), &raw_value);
  if (ret != VSX_STATUS_SUCCESS)
  {
    NODELET_ERROR_STREAM("Could not read parameter: " << ret);
  }
  auto param_value = std::string(raw_value);

  NODELET_INFO_STREAM("Read parameter from camera: " << parameter_id << ": " << param_value);

  vsx_ReleaseString(&raw_value);
  return func(param_value);
}

void Vte7500Driver::enumerateSettings(VsxSystemHandle **vsx_system_handle)
{
  VsxParameterList *list = nullptr;

  VsxStatusCode ret;

  ret = vsx_GetParameterList(*vsx_system_handle, &list);
  if (ret != VSX_STATUS_SUCCESS)
  {
    NODELET_ERROR_STREAM("Could not get parameter list: " << ret);
    return;
  }

  NODELET_INFO_STREAM("Found " << list->length << " parameters.");

  for (size_t i = 0; i < list->length; ++i)
  {
    const auto& param = list->parameters[i];

    NODELET_INFO_STREAM("Name: " << param.name
                        << " configId: " <<  param.configId
                        << " configVersion: " << param.configVersion
                        << " parameterId: " << list->parameters[i].parameterId
                        << " settingsVersion: " << list->parameters[i].settingsVersion);

    for (size_t j = 0; j < param.enumItemListLength; ++j)
    {
      const auto& enum_item = param.enumItemList[j];
      NODELET_INFO_STREAM("\tname: " << enum_item.name << " id: " << enum_item.id);
    }
  }

  vsx_ReleaseParameterList(&list);
}

void Vte7500Driver::initializeReconfigure()
{
  Vte7500DriverConfig config;

  config.autotrigger_frame_rate = readParamFromDevice<int>(1, "Base", 1, "AutoTriggerFrameRate",[](std::string_view input) {
    std::stringstream ss {input.data()};
    int result;
    ss >> result;
    return result;
  });

  config.binning_mode  = readParamFromDevice<int>(1, "Base", 1, "BinningMode", [](std::string_view input) {
    if (input == "2x2")
    {
      return 0;
    }
    else if (input == "4x4")
    {
      return 1;
    }
    else if (input == "Off")
    {
      return 2;
    }
    else
    {
      return -1;
    }
  });

  config.camera_mode = readParamFromDevice<int>(1, "Base", 1, "CameraMode", [](std::string_view input) {
    if (input == "FastMovement")
    {
      return 0;
    }
    else if (input == "HighDynamic")
    {
      return 1;
    }
    else if (input == "Standard")
    {
      return 2;
    }
    else
    {
      return -1;
    }
  });

  config.exposure_time = readParamFromDevice<int>(1, "Base", 1, "ExposureTime",[](std::string_view input) {
    std::stringstream ss {input.data()};
    int result;
    ss >> result;
    return result;
  });

  config.flying_pixel_filter = readParamFromDevice<bool>(1, "Base", 1, "FlyingPixelFilterEnabled",[](std::string_view input) {
    /*
     * TODO: This logic does not appropriately tell the user if the input is something other than "True" or "False".
     */
    return (input == "True");
  });

  config.range_mode = readParamFromDevice<int>(1, "Base", 1, "RangeMode", [](std::string_view input) {
    if (input == "1500")
    {
      return 0;
    }
    else if (input == "3750")
    {
      return 1;
    }
    else if (input == "7500")
    {
      return 2;
    }
    else
    {
      return -1;
    }
  });

  config.spatial_filter = readParamFromDevice<bool>(1, "Base", 1, "SpatialFilterEnabled",[](std::string_view input) {
    /*
     * TODO: This logic does not appropriately tell the user if the input is something other than "True" or "False".
     */
    return (input == "True");
  });

  {
    boost::recursive_mutex::scoped_lock lock(reconfigure_mutex_);
    reconfigure_server_->updateConfig(config);
  }

}

void Vte7500Driver::reconfigureCallback(Vte7500DriverConfig &config, uint32_t level)
{
  VsxStatusCode ret = VSX_STATUS_SUCCESS;

  NODELET_INFO_STREAM("Starting Reconfigure");

  ret = vsx_SetSingleParameterValue(vsx_system_handle_, 1, "Base", 1, "AutoTriggerFrameRate", std::to_string(config.autotrigger_frame_rate).data());
  if (ret != VSX_STATUS_SUCCESS)
  {
    NODELET_ERROR_STREAM("Could not reconfigure AutoTriggerFrameRate.");
  }

  std::string binning_mode;
  switch (config.binning_mode)
  {
    case 0:
      binning_mode = "2x2";
      break;
    case 1:
      binning_mode = "4x4";
      break;
    case 2:
      binning_mode = "Off";
      break;
    default:
      binning_mode = "Off";
  }
  ret = vsx_SetSingleParameterValue(vsx_system_handle_, 1, "Base", 1, "BinningMode", binning_mode.data());
  if (ret != VSX_STATUS_SUCCESS)
  {
    NODELET_ERROR_STREAM("Could not reconfigure BinningMode.");
  }

  std::string camera_mode;
  switch (config.camera_mode)
  {
    case 0:
      camera_mode = "FastMovement";
      break;
    case 1:
      camera_mode = "HighDynamic";
      break;
    default: // 2
      camera_mode = "Standard";
  }
  ret = vsx_SetSingleParameterValue(vsx_system_handle_, 1, "Base", 1, "CameraMode", camera_mode.data());
  if (ret != VSX_STATUS_SUCCESS)
  {
    NODELET_ERROR_STREAM("Could not reconfigure CameraMode.");
  }

  ret = vsx_SetSingleParameterValue(vsx_system_handle_, 1, "Base", 1, "ExposureTime", std::to_string(config.exposure_time).data());
  if (ret != VSX_STATUS_SUCCESS)
  {
    NODELET_ERROR_STREAM("Could not reconfigure ExposureTime.");
  }

  std::string flying_pixel_filter = (config.flying_pixel_filter) ? "True" : "False";
  ret = vsx_SetSingleParameterValue(vsx_system_handle_, 1, "Base", 1, "FlyingPixelFilterEnabled", flying_pixel_filter.data());
  if (ret != VSX_STATUS_SUCCESS)
  {
    NODELET_ERROR_STREAM("Could not reconfigure FlyingPixelFilterEnabled.");
  }

  std::string range_mode;
  switch (config.range_mode)
  {
    case 0:
      range_mode = "1500";
      break;
    case 1:
      range_mode = "3750";
      break;
    default: // 2
      range_mode = "7500";
  }
  ret = vsx_SetSingleParameterValue(vsx_system_handle_, 1, "Base", 1, "RangeMode", range_mode.data());
  if (ret != VSX_STATUS_SUCCESS)
  {
    NODELET_ERROR_STREAM("Could not reconfigure RangeMode.");
  }

  std::string spatial_filter = (config.spatial_filter) ? "True" : "False";
  ret = vsx_SetSingleParameterValue(vsx_system_handle_, 1, "Base", 1, "SpatialFilterEnabled", spatial_filter.data());
  if (ret != VSX_STATUS_SUCCESS)
  {
    NODELET_ERROR_STREAM("Could not reconfigure SpatialFilterEnabled.");
  }

  ret = vsx_ResetDynamicContainerGrabber(vsx_system_handle_, 2, VSX_STRATEGY_DROP_OLDEST);

  NODELET_INFO_STREAM("Finished Reconfigure");
}


void Vte7500Driver::run()
{
  auto nh = getMTNodeHandle();
  auto pnh = getMTPrivateNodeHandle();

  NODELET_INFO_STREAM("Starting VTE 7500 Driver.");

  /*
   * Connect to sensor.
   */
  VsxStatusCode ret;
  VsxDeviceList *device_list = nullptr;

  ret = vsx_GetUdpDeviceList(&device_list);
  if (ret != VSX_STATUS_SUCCESS)
  {
    NODELET_FATAL_STREAM("Could not get UDP device list: " << ret);
    return;
  }
  else if (device_list->length > 0)
  {
    VsxDevice dev = device_list->devices[0];
    NODELET_INFO_STREAM("Device found: " << dev.sensorType << " " << dev.ipAddress);
    ret = vsx_InitTcpSensor(&vsx_system_handle_, dev.ipAddress, "");
  }
  else
  {
    /* Use fixed IP address */
    ret = vsx_InitTcpSensor(&vsx_system_handle_, "192.168.2.4", "");
  }
  vsx_ReleaseDeviceList(&device_list);

  ret = vsx_Connect(vsx_system_handle_);
  if (ret != VSX_STATUS_SUCCESS)
  {
    NODELET_FATAL_STREAM("Could not connect to sensor: " << ret);
    ret = vsx_ReleaseSensor(&vsx_system_handle_);
    return;
  }

  /*
   * Read parameters.
   */
  Vte7500DriverConfig config;
  config.range_mode = pnh.param("range_mode", 2);
  config.camera_mode = pnh.param("camera_mode", 2);
  config.binning_mode = pnh.param("binning_mode", 0);
  config.flying_pixel_filter = pnh.param("flying_pixel_filter", true);
  config.spatial_filter = pnh.param("spatial_filter", true);
  config.exposure_time = pnh.param("exposure_time", 1000);
  config.autotrigger_frame_rate = pnh.param("auto_trigger_frame_rate", 30);
  reconfigureCallback(config, 0);

  pnh.getParam("frame", tf_frame_);


  /*
   * Setup dynamic reconfigure.
   * Load default parameters to clear any bad configuration.
   */
  ret = vsx_LoadDefaultParameterSetOnDevice(vsx_system_handle_);
  enumerateSettings(&vsx_system_handle_);

  reconfigure_server_ = std::make_unique<ReconfigureServerVte7500>(reconfigure_mutex_, pnh);
  initializeReconfigure();
  f_ = boost::bind(&Vte7500Driver::reconfigureCallback, this, _1, _2);
  reconfigure_server_->setCallback(f_);

  /*
   * Setup publishers.
   */
  cloud_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("cloud", 2);

  NODELET_INFO_STREAM("Starting capture from sensor");

  vsx_SetSingleParameterValue(vsx_system_handle_, 1, "Base", 1, "TriggerSource", "AutoTrigger");
  vsx_SetSingleParameterValue(vsx_system_handle_, 1, "Base", 1, "TriggerEnabled", "1");
  vsx_SetSingleParameterValue(vsx_system_handle_, 1, "Base", 1, "OutputMode", "CalibratedABC_Grid_Amp");

  ret = vsx_ResetDynamicContainerGrabber(vsx_system_handle_, 2, VSX_STRATEGY_DROP_OLDEST);

  while(ros::ok())
  {
    VsxDataContainerHandle *dch = nullptr;
    VsxImage *line_data = nullptr;
    VsxImage *line_datab = nullptr;
    VsxImage *lin_datac = nullptr;
    VsxImage *linedatai = nullptr;

    ret = vsx_GetDataContainer(vsx_system_handle_, &dch, 1000);
    if (ret != VSX_STATUS_SUCCESS)
    {
      NODELET_ERROR_STREAM_THROTTLE(1.0, "Waiting for sensor: " << ret);
      continue;
    }

    ret = vsx_GetImage(dch, "CalibratedA", &line_data);
    if (ret != VSX_STATUS_SUCCESS)
    {
      NODELET_ERROR_STREAM("HIH");
      continue;
    }
    ret = vsx_GetImage(dch, "CalibratedB", &line_datab);
    if (ret != VSX_STATUS_SUCCESS)
    {
      NODELET_ERROR_STREAM("HIH");
      continue;
    }
    ret = vsx_GetImage(dch, "CalibratedC", &lin_datac);
    if (ret != VSX_STATUS_SUCCESS)
    {
      NODELET_ERROR_STREAM("HIH");
      continue;
    }
    ret = vsx_GetImage(dch, "Amplitude", &linedatai);
    if (ret != VSX_STATUS_SUCCESS)
    {
      NODELET_ERROR_STREAM("HIH");
      continue;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr msg(new pcl::PointCloud<pcl::PointXYZI>);
    msg->header.frame_id = tf_frame_;
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);

    std::vector<int16_t> X;
    vsxImageToVec(line_data, X);

    std::vector<int16_t> Y;
    vsxImageToVec(line_datab, Y);

    std::vector<int16_t> Z;
    vsxImageToVec(lin_datac, Z);

    std::vector<uint16_t> I;
    vsxImageToVec(linedatai, I);

/*
 * TODO: Figure out how to directly copy data into PCL point cloud.
 */
//  auto mapX = msg->getMatrixXfMap(1, 4,0);
//  auto mapy = msg->getMatrixXfMap(1, 4,1);
//  auto mapZ = msg->getMatrixXfMap(1, 4,2);
//  auto mapI= msg->getMatrixXfMap(1, 4,3);

    for (size_t i = 0; i < X.size() / 2; ++i)
    {
      pcl::PointXYZI p;
      p.x = static_cast<float>(X[i]) / 1000.0f;
      p.y = static_cast<float>(Y[i]) / 1000.0f;
      p.z = static_cast<float>(Z[i]) / 1000.0f;
      p.intensity = static_cast<float>(I[i])/ 65535.0f;
      if (Z[i] != -32768)
        msg->points.push_back(p);
    }

    cloud_pub_.publish(msg);

    vsx_ReleaseImage(&line_data);
    vsx_ReleaseImage(&line_datab);
    vsx_ReleaseImage(&lin_datac);
    vsx_ReleaseImage(&linedatai);
    vsx_ReleaseDataContainer(&dch);
  }

  NODELET_INFO_STREAM("Releasing sensor;");
  ret = vsx_ReleaseSensor(&vsx_system_handle_);
}
} // vsx_protocol_driver_ros

#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(vsx_protocol_driver_ros, Vte7500Driver)