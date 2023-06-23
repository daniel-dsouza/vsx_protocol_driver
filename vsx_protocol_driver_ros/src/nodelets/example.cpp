//
// Created by ddsouza on 6/23/23.
//

#include "vsx_protocol_driver_ros/example.h"
#include "vsx_protocol_driver/PF.VsxProtocolDriver.WrapperNE.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

void test_pcl()
{




}

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

void Example::run()
{
  auto nh = getMTPrivateNodeHandle();

  cloud_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("cloud", 2);

  NODELET_INFO_STREAM("running sample nodelet");

  VsxDeviceList *device_list = nullptr;
  VsxStatusCode ret;
  VsxSystemHandle *vsx = nullptr;

  ret = vsx_GetUdpDeviceList(&device_list);

  if (ret != VSX_STATUS_SUCCESS)
  {
    NODELET_FATAL_STREAM("HUH");
  }
  else if (device_list->length > 0)
  {
    VsxDevice dev = device_list->devices[0];
    NODELET_INFO_STREAM("Device found: " << dev.sensorType << " " << dev.ipAddress);
    ret = vsx_InitTcpSensor(&vsx, dev.ipAddress, "");
  }
  else
  {
    /* Use fixed IP address */
    ret = vsx_InitTcpSensor(&vsx, "192.168.2.4", "");
  }

  ret = vsx_Connect(vsx);
  if (ret != VSX_STATUS_SUCCESS)
  {
    NODELET_ERROR_STREAM(ret);
    ret = vsx_ReleaseSensor(&vsx);
  }

  vsx_SetSingleParameterValue(vsx, 1, "Base", 1, "TriggerSource", "AutoTrigger");
  vsx_SetSingleParameterValue(vsx, 1, "Base", 1, "TriggerEnabled", "1");
  vsx_SetSingleParameterValue(vsx, 1, "Base", 1, "OutputMode", "CalibratedABC_Grid_Amp");

  ret = vsx_ResetDynamicContainerGrabber(vsx, 2, VSX_STRATEGY_DROP_OLDEST);

  while(true)
  {
    VsxDataContainerHandle *dch = NULL;
    ret = vsx_GetDataContainer(vsx, &dch, 1000);

    VsxImage *line_data = nullptr, *line_datab = nullptr, *lin_datac = nullptr, *linedatai = nullptr;

    std::cout << "getimageA: " << vsx_GetImage(dch, "CalibratedA", &line_data) << "\n";
    std::cout << "getimageB: " << vsx_GetImage(dch, "CalibratedB", &line_datab) << "\n";
    std::cout << "getimageC: " << vsx_GetImage(dch, "CalibratedC", &lin_datac) << "\n";
    std::cout << "getimageI: " << vsx_GetImage(dch, "Amplitude", &linedatai) << "\n";

    std::cout << "format: " << line_data->format << "\n";

    std::cout << "width: " << line_data->width << " height: " << line_data->height << "\n";

    pcl::PointCloud<pcl::PointXYZI>::Ptr msg(new pcl::PointCloud<pcl::PointXYZI>);
//    msg->height = 120;
//    msg->width = 160;
    msg->header.frame_id = "base_link";
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
//    msg->points.reserve(120 * 160);

    std::vector<int16_t> X;
    vsxImageToVec(line_data, X);

    std::vector<int16_t> Y;
    vsxImageToVec(line_datab, Y);

    std::vector<int16_t> Z;
    vsxImageToVec(lin_datac, Z);

    std::vector<uint16_t> I;
    vsxImageToVec(linedatai, I);

    vsx_ReleaseDataContainer(&dch);

    NODELET_INFO_STREAM("vuc_size: " << X.size());



//  auto mapX = msg->getMatrixXfMap(1, 4,0);
//  auto mapy = msg->getMatrixXfMap(1, 4,1);
//  auto mapZ = msg->getMatrixXfMap(1, 4,2);
//  auto mapI= msg->getMatrixXfMap(1, 4,3);

//  ROS_INFO_STREAM("rows " << mapX.rows() << " cols " << mapX.cols());

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

    NODELET_INFO_STREAM("big mood");
  }



//  ret = vsx_ReleaseSensor(&vsx);
}
} // vsx_protocol_driver_ros

#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(vsx_protocol_driver_ros, Example)