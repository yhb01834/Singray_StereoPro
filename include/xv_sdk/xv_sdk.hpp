#pragma once

#include <memory>
#include <map>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include <xv-sdk.h>

// ROS

#include <ros/ros.h>

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Time.h>
#include <std_msgs/String.h>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include <std_srvs/Trigger.h>

#include <xv_sdk/GetPose.h>
#include <xv_sdk/GetPoseAt.h>
#include <xv_sdk/GetOrientation.h>
#include <xv_sdk/GetOrientationAt.h>
#include <xv_sdk/GetDevices.h>

//#define NOT_USE_RGB
#define NOT_USE_TOF
//#define NOT_USE_SGBM
#define NOT_USE_FE
//#define USE_SLAM_PATH
#define USE_SLAM_POSE
//#define USE_MAPPING_ON_HOST

#ifdef USE_SLAM_POSE
    #define ENABLE_POSE_WITH_CONFIDENCE
#endif

//#define ENABLE_SLAM_POSE_FACTOR_CONVERT
//#define ENABLE_INFO_PRINT

#ifndef NOT_USE_TOF
//  #define TOF_QVGA
  #ifndef NOT_USE_RGB
    #define TOF_PC_WITH_RGB
  #endif/*NOT_USE_RGB*/
#endif/*#ifndef NOT_USE_TOF*/

#ifndef NOT_USE_SGBM
  #define USE_SGBM_POINTCLOUD
  //#define USE_SGBM_IMAGE

  #ifdef USE_SGBM_POINTCLOUD
    //#define DISPLAY_POINT_CLOUD_FPS
    #define SGBM_POINTCLOUD_UNIT_M
    //#define CONVERT_TO_WORLD_COORDINATE_SYSTEM
    //#define SGBM_PC_WITH_RGB
    #define SGBM_FIRMWARE_CLOUD
    #define ENABLE_LESS_POINT_CLOUD
  #endif/*#ifndef USE_SGBM_POINTCLOUD*/
#endif/*#ifndef NOT_USE_SGBM*/

namespace xv {

struct FpsCount
{
  std::vector<std::chrono::system_clock::time_point> frames_ts;
  long long total = 0;

  void tic()
  {
    frames_ts.push_back( std::chrono::system_clock::now() );
    ++total;
    while( std::chrono::duration_cast<std::chrono::milliseconds>( frames_ts.back() - frames_ts.front() ).count() > 1100 ){
      frames_ts.erase( frames_ts.begin() );
    }
  }

  double fps()
  {
    double fps = 0;
    const size_t &size = frames_ts.size();
    if( size > 2 ){
      auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>( frames_ts.back() - frames_ts.front() ).count();
      fps = 1000000.0 * static_cast<double>(size - 1) / microseconds;
    }
    return fps;
  }

  void reset()
  {
      frames_ts.clear();
      total = 0;
  }
};

class RosDevice
{
private:
  ros::NodeHandle m_nodeHandle;
  std::string m_rosNamespace;
  std::shared_ptr<Device> m_xvDevice;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> m_ddynReconfig;
  std::string m_param_tfPrefix;

  /**
   * SLAM
   */

  ros::NodeHandle m_nodeHandle_slam;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> m_ddynReconfig_slam;
  bool m_param_slam_autoStart = true;
  ros::ServiceServer m_server_slam_start;
  ros::ServiceServer m_server_slam_stop;
  ros::ServiceServer m_server_slam_getPose;
  ros::ServiceServer m_server_slam_getPoseAt;
  ros::Publisher m_publisher_slam_pose;
  ros::Publisher m_publisher_slam_visualPose;
  //ros::Publisher m_publisher_slam_lost;
  ros::Publisher m_publisher_slam_stereoPlanes;
  ros::Publisher m_publisher_slam_stereoPlanesMarkers;
  ros::Publisher m_publisher_slam_tofPlanes;
  ros::Publisher m_publisher_slam_tofPlanesMarkers;
  ros::Publisher m_publisher_slam_mapPoints;
  ros::Publisher m_publisher_slam_keyFrames;
  ros::Publisher m_publisher_slam_path;

  /**
   * EDGE
   */

  ros::NodeHandle m_nodeHandle_edge;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> m_ddynReconfig_edge;
  bool m_param_edge_autoStart = false;
  ros::ServiceServer m_server_edge_start;
  ros::ServiceServer m_server_edge_stop;
  ros::Publisher m_publisher_edge_pose;
  ros::Publisher m_publisher_edge_lost;

  /**
   * IMU
   */

  ros::NodeHandle m_nodeHandle_imuSensor;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> m_ddynReconfig_imuSensor;
  double m_param_imuSensor_linear_acceleration_stddev = 0.;
  double m_param_imuSensor_angular_velocity_stddev = 0.;
  //double m_param_imuSensor_magnetic_field_stddev = 0.;
  double m_param_imuSensor_orientation_stddev = 0.;
  ros::ServiceServer m_server_imuSensor_startOri;
  ros::ServiceServer m_server_imuSensor_stopOri;
  ros::ServiceServer m_server_imuSensor_getOri;
  ros::ServiceServer m_server_imuSensor_getOriAt;
  ros::Publisher m_publisher_imuSensor_dataRaw;
  ros::Publisher m_publisher_imuSensor_orientation;

  /**
   * Fisheyes
   */

  ros::NodeHandle m_nodeHandle_fisheye;
  ros::NodeHandle m_nodeHandle_fisheyeLeft;
  ros::NodeHandle m_nodeHandle_fisheyeRight;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> m_ddynReconfig_fisheye;
  bool m_param_fisheye_autoExposure = true;
  //ros::Publisher m_publisher_fisheyeCameras_images;
  //ros::Publisher m_publisher_fisheyeCameras_left;
  //ros::Publisher m_publisher_fisheyeCameras_right;
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_camInfoMan_fisheye_left;
  std::shared_ptr<image_transport::ImageTransport> m_imageTransport_fisheye_left;
  image_transport::CameraPublisher m_publisher_fisheyeCameras_leftImage;
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_camInfoMan_fisheye_right;
  std::shared_ptr<image_transport::ImageTransport> m_imageTransport_fisheye_right;
  image_transport::CameraPublisher m_publisher_fisheyeCameras_rightImage;
  std::vector<std::map<int /*height*/, sensor_msgs::CameraInfo>> m_fisheyeCameraInfos;
  std::vector<Calibration> m_xvFisheyesCalibs;

  /**
   * RGB
   */

  ros::NodeHandle m_nodeHandle_colorCamera;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> m_ddynReconfig_colorCamera;
  ros::ServiceServer m_server_colorCamera_start;
  ros::ServiceServer m_server_colorCamera_stop;
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_camInfoMan_colorCamera;
  std::shared_ptr<image_transport::ImageTransport> m_imageTransport_colorCamera;
  image_transport::CameraPublisher m_publisher_colorCamera_imageColor;
  std::map<int, sensor_msgs::CameraInfo> m_colorCameraInfos;
  Calibration m_xvColorCalib;

  /**
   * SGBM
   */

  ros::NodeHandle m_nodeHandle_sgbmCamera;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> m_ddynReconfig_sgbmCamera;
  ros::ServiceServer m_server_sgbmCamera_start;
  ros::ServiceServer m_server_sgbmCamera_stop;
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_camInfoMan_sgbmCamera;
  std::shared_ptr<image_transport::ImageTransport> m_imageTransport_sgbmCamera;
  image_transport::CameraPublisher m_publisher_sgbmCamera_image;
  std::map<int, sensor_msgs::CameraInfo> m_sgbmCameraInfos;
  Calibration m_xvSgbmCalib;

  /**
   * ToF
   */

  ros::NodeHandle m_nodeHandle_tofCamera;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> m_ddynReconfig_tofCamera;
  bool m_param_tofCamera_useMapFrame = true; /// if false, the point cloud is expressed in local ToF frame
  ros::ServiceServer m_server_tofCamera_start;
  ros::ServiceServer m_server_tofCamera_stop;
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_camInfoMan_tofCamera;
  std::shared_ptr<image_transport::ImageTransport> m_imageTransport_tofCamera;
  image_transport::CameraPublisher m_publisher_tofCamera_image;
  Calibration m_xvTofCalib;

  /**
   * RGBD
   */

  ros::ServiceServer m_server_rgbd_start;
  ros::ServiceServer m_server_rgbd_stop;
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_camInfoMan_rgbd;
  std::shared_ptr<image_transport::ImageTransport> m_imageTransport_rgbd;
  image_transport::CameraPublisher m_publisher_rgbd_image;

  /**
   * Color for ToF
   */

  std::mutex m_lastColorMtx;
  ColorImage m_last_xvColorImage;
  std::shared_ptr<image_transport::ImageTransport> m_imageTransport_virtual_colorForToF;
  image_transport::CameraPublisher m_publisher_tofCamera_imageColor;
  ros::Publisher m_publisher_tofCamera_pointCloud;

  /**
   * Color for Sgbm
   */
  std::shared_ptr<image_transport::ImageTransport> m_imageTransport_virtual_colorForSgbm;
  bool m_param_sgbmCamera_useMapFrame = true; /// if false, the point cloud is expressed in local sgbm frame
  image_transport::CameraPublisher m_publisher_sgbmCamera_imageColor;
  ros::Publisher m_publisher_sgbmCamera_pointCloud;

public:
  RosDevice(ros::NodeHandle *parent, const std::string& rosNamespace, std::shared_ptr<Device> device);

  std::string getRosNamespace() const;

private:

  std::string getFrameId(const std::string& defaultId) const;

  void init();
  void initPublisherAdvertise(void);

  bool cbSlam_start(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool cbSlam_stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool cbSlam_getPose(xv_sdk::GetPose::Request &req, xv_sdk::GetPose::Response &res);
  bool cbSlam_getPoseAt(xv_sdk::GetPoseAt::Request &req, xv_sdk::GetPoseAt::Response &res);

  bool cbImuSensor_startOri(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool cbImuSensor_stopOri(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool cbImuSensor_getOri(xv_sdk::GetOrientation::Request &req, xv_sdk::GetOrientation::Response &res);
  bool cbImuSensor_getOriAt(xv_sdk::GetOrientationAt::Request &req, xv_sdk::GetOrientationAt::Response &res);

  bool cbColorCamera_start(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool cbColorCamera_stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool cbSgbmCamera_start(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool cbSgbmCamera_stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool cbTofCamera_start(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool cbTofCamera_stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool cbRgbd_start(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool cbRgbd_stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  friend void showSgbmDepthImage(const RosDevice* device, const SgbmImage & xvSgbmImage);
  friend void showSgbmPointCloudImage(RosDevice* device, const SgbmImage & xvSgbmImage);
};

class RosWrapper
{
private:
  ros::NodeHandle* m_topNodeHandle;

  bool m_stopWatchDevices;
  std::thread m_watchDevicesThread;
  std::mutex m_deviceMapMutex;
  std::map<std::string, std::shared_ptr<RosDevice>> m_deviceMap;
  void watchDevices();

  ros::ServiceServer m_server_getDevices;
  ros::Publisher m_publisher_newDevice;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> m_ddynReconfig;
  int m_uuidLength = 0;

public:
  RosWrapper(ros::NodeHandle *nh);

  ~RosWrapper();

  bool cbGetDevices(xv_sdk::GetDevices::Request &req, xv_sdk::GetDevices::Response &res);

  void publishNewDevice(const std::string &rosNamespace);
};

} /// namespace xv
