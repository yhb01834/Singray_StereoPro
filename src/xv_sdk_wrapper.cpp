#include <xv_sdk/xv_sdk.hpp>

#include <regex>
#include <fstream>
#include <sstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/MarkerArray.h>
#include <xv_sdk/FisheyeImages.h>
#include <xv_sdk/OrientationStamped.h>
#include <xv_sdk/Plane.h>
#include <xv_sdk/Planes.h>
#include <xv_sdk/Lost.h>
#include <xv_sdk/PoseStampedConfidence.h>
#include <xv_sdk/xv-sdk-private.h>

namespace xv {

std::array<double, 4> rotationToQuaternion(const std::array<double, 9> &rot);
bool raytrace(const PolynomialDistortionCameraModel& intrinsic, const Vector2d& pt, Vector3d& ray);
bool project(Transform const& pose, Transform const& extrinsic, const PolynomialDistortionCameraModel& intrinsic, Vector3d const& p3d, Vector2d& p2d);
bool raytrace(const UnifiedCameraModel& intrinsic, const Vector2d& pt, Vector3d& ray);
bool project(Transform const& pose, Transform const& extrinsic, const UnifiedCameraModel& intrinsic, Vector3d const& p3d, Vector2d& p2d);
xv_sdk::PoseStampedConfidence toRosPoseStampedConfidence(const Pose& xvPose, const std::string& frame_id);

using CamInfoM = camera_info_manager::CameraInfoManager;
using ImgTpt = image_transport::ImageTransport;
enum pointCloudUnit{m,mm};
namespace {

double steady_clock_now()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()*1e-9;
}

/// see https://www.ros.org/reps/rep-0103.html
/// "_optical_frame" indicates the following convention: z forward, x right, y down
/// "_frame" alone indicates the following convention: x forward, y left, z up
static std::map<std::string, std::string> s_frameIds = {
  {"map_optical_frame","map_optical_frame"}, /// the "world" with possible localization jumps
  {"imu_optical_frame","imu_optical_frame"},
  {"imu_optical_frame_unfiltered","imu_optical_frame_unfiltered"},
  {"fisheye_left_optical_frame","fisheye_left_optical_frame"},
  {"fisheye_right_optical_frame","fisheye_right_optical_frame"},
  {"color_optical_frame","color_optical_frame"},
  {"sgbm_optical_frame","sgbm_optical_frame"},
  {"tof_optical_frame","tof_optical_frame"},
  {"imu_link","imu_link"},
  {"base_link","base_link"},
  {"odom","odom"},
};
static std::mutex s_paramsMutex;
#define GUARD_PARAMS std::lock_guard<std::mutex> lock(s_paramsMutex);

static std::shared_ptr<tf2_ros::TransformBroadcaster> s_tfBroadcaster;
static std::shared_ptr<tf2_ros::StaticTransformBroadcaster> s_tfStaticBroadcaster;
static nav_msgs::Path path_msgs;

inline geometry_msgs::Vector3 toRosVector3(const Vector3d& vec3)
{
  geometry_msgs::Vector3 v;
  v.x = vec3[0];
  v.y = vec3[1];
  v.z = vec3[2];
  return v;
}

inline geometry_msgs::Point toRosPoint(const Vector3d& vec3)
{
  geometry_msgs::Point p;
  p.x = vec3[0];
  p.y = vec3[1];
  p.z = vec3[2];
  return p;
}

geometry_msgs::Pose toRosPose(const Transform& xvTf)
{
  /// see https://www.ros.org/reps/rep-0105.html
  geometry_msgs::Pose pose;

  pose.position.x = xvTf.x();
  pose.position.y = xvTf.y();
  pose.position.z = xvTf.z();

  const auto quat = rotationToQuaternion(xvTf.rotation()); /// [qx,qy,qz,qw]
  pose.orientation.x = quat[0];
  pose.orientation.y = quat[1];
  pose.orientation.z = quat[2];
  pose.orientation.w = quat[3];

  return pose;
}

geometry_msgs::Pose toRosPose(const Pose& xvPose)
{
  /// see https://www.ros.org/reps/rep-0105.html
  geometry_msgs::Pose pose;

  pose.position.x = xvPose.x();
  pose.position.y = xvPose.y();
  pose.position.z = xvPose.z();

  const auto quat = xvPose.quaternion(); /// [qx,qy,qz,qw]
  pose.orientation.x = quat[0];
  pose.orientation.y = quat[1];
  pose.orientation.z = quat[2];
  pose.orientation.w = quat[3];

  return pose;
}

geometry_msgs::PoseStamped toRosPoseStamped(const Pose& xvPose, const std::string& frame_id)
{
    geometry_msgs::PoseStamped ps;
    static double old_timeStamp = steady_clock_now();
    if (xvPose.hostTimestamp() < 0)
      std::cerr << "XVSDK-ROS-WRAPPER toRosPoseStamped() Error: negative Pose host-timestamp" << std::endl;
    try
    {
        double currentTimestamp = xvPose.hostTimestamp() > 0.1 ? xvPose.hostTimestamp() : 0.1;
        ps.header.stamp.fromSec(currentTimestamp);
        old_timeStamp = currentTimestamp;
    }
    catch(std::runtime_error& ex) 
    {
        ps.header.stamp.fromSec(old_timeStamp);
    }
    ps.header.frame_id = frame_id;

    ps.pose.position.x = xvPose.x();
    ps.pose.position.y = xvPose.y();
    ps.pose.position.z = xvPose.z();

    const auto quat = xvPose.quaternion(); /// [qx,qy,qz,qw]
    ps.pose.orientation.x = quat[0];
    ps.pose.orientation.y = quat[1];
    ps.pose.orientation.z = quat[2];
    ps.pose.orientation.w = quat[3];

    return ps;
}

nav_msgs::Path toRosPoseStampedRetNavmsgs(const Pose& xvPose, const std::string& frame_id,nav_msgs::Path& ps)
{
  if (xvPose.hostTimestamp() < 0)
    std::cerr << "XVSDK-ROS-WRAPPER toRosPoseStamped() Error: negative Pose host-timestamp" << std::endl;
  
  ros::Time current_time,last_time;
  /// see https://www.ros.org/reps/rep-0105.html
  // nav_msgs::Path ps;
  // current_time = ros::Time::now();
  // ps.header.stamp = current_time;//.fromSec(xvPose.hostTimestamp() > 0.1 ? xvPose.hostTimestamp() : 0.1);
  // ps.header.frame_id = frame_id;

  geometry_msgs::PoseStamped this_ps;
  this_ps.header.frame_id = frame_id;
  this_ps.header.stamp.fromSec(xvPose.hostTimestamp() > 0.1 ? xvPose.hostTimestamp() : 0.1);
  this_ps.pose.position.x = xvPose.x();
  this_ps.pose.position.y = xvPose.y();
  this_ps.pose.position.z = xvPose.z();

  const auto quat = xvPose.quaternion(); /// [qx,qy,qz,qw]
  this_ps.pose.orientation.x = quat[0];
  this_ps.pose.orientation.y = quat[1];
  this_ps.pose.orientation.z = quat[2];
  this_ps.pose.orientation.w = quat[3];
  
  ps.poses.push_back(this_ps);

  return ps;
}

geometry_msgs::TransformStamped toRosTransformStamped(const Pose& pose, const std::string& parent_frame_id, const std::string& frame_id)
{
    geometry_msgs::TransformStamped tf;
    static double old_timeStamp = steady_clock_now();
    if (pose.hostTimestamp() < 0)
    {
        std::cerr << "XVSDK-ROS-WRAPPER toRosTransformStamped() Error: negative Pose host-timestamp" << std::endl;
    }
    
    try
    {
      double currentTimestamp = pose.hostTimestamp() > 0.1 ? pose.hostTimestamp() : 0.1;
      tf.header.stamp.fromSec(currentTimestamp);
      old_timeStamp = currentTimestamp;
    }
    catch(std::runtime_error& ex) 
    {
        tf.header.stamp.fromSec(old_timeStamp);    
    }
    tf.header.frame_id = parent_frame_id;
    tf.child_frame_id = frame_id;

    tf.transform.translation.x = pose.x();
    tf.transform.translation.y = pose.y();
    tf.transform.translation.z = pose.z();

    auto quat = pose.quaternion(); /// [qx,qy,qz,qw]
    tf.transform.rotation.x = quat[0];
    tf.transform.rotation.y = quat[1];
    tf.transform.rotation.z = quat[2];
    tf.transform.rotation.w = quat[3];
    
    return tf;
}

xv_sdk::OrientationStamped toRosOrientationStamped(Orientation const& xvOrientation, const std::string& frame_id)
{
  if (xvOrientation.hostTimestamp < 0)
    std::cerr << "XVSDK-ROS-WRAPPER toRosOrientationStamped() Error: negative Orientation host-timestamp" << std::endl;

  xv_sdk::OrientationStamped orientation;
  orientation.header.stamp.fromSec(xvOrientation.hostTimestamp > 0.1 ? xvOrientation.hostTimestamp : 0.1);
  orientation.header.frame_id = frame_id;

  for (int i = 0; i < 9; ++i)
    orientation.matrix[i] = xvOrientation.rotation()[i];

  auto quat = xvOrientation.quaternion(); /// [qx,qy,qz,qw]
  orientation.quaternion.x = quat[0];
  orientation.quaternion.y = quat[1];
  orientation.quaternion.z = quat[2];
  orientation.quaternion.w = quat[3];

  orientation.angularVelocity.x = xvOrientation.angularVelocity()[0];
  orientation.angularVelocity.y = xvOrientation.angularVelocity()[1];
  orientation.angularVelocity.z = xvOrientation.angularVelocity()[2];

  return orientation;
}

sensor_msgs::Image toRosImage(const GrayScaleImage& xvGrayImage, double timestamp, const std::string& frame_id)
{
  if (timestamp < 0)
    std::cerr << "XVSDK-ROS-WRAPPER toRosImage() Error: negative timestamp" << std::endl;

  sensor_msgs::Image rosImage;
  rosImage.header.stamp.fromSec(timestamp > 0.1 ? timestamp : 0.1);
  rosImage.header.frame_id = frame_id;
  rosImage.height = xvGrayImage.height;
  rosImage.width = xvGrayImage.width;
  rosImage.encoding = sensor_msgs::image_encodings::MONO8;
  rosImage.is_bigendian = false;
  rosImage.step = xvGrayImage.width * sizeof(uint8_t); /// bytes for 1 line
  /// TODO How to avoid copy ?
  const int nbPx = xvGrayImage.width * xvGrayImage.height;
  std::vector<uint8_t> copy(nbPx);
  std::memcpy(&copy[0], xvGrayImage.data.get(), nbPx * sizeof(uint8_t));
  rosImage.data = std::move(copy);

  return rosImage;
}

cv::Mat toCvMatRGB(const ColorImage& xvColorImage)
{
  cv::Mat cvMat;
  switch(xvColorImage.codec){
     case ColorImage::Codec::YUYV:{
         cv::Mat img( xvColorImage.height, xvColorImage.width, CV_8UC2, (void*)xvColorImage.data.get() );
         cv::cvtColor( img, cvMat, cv::COLOR_YUV2RGB_YUYV );
         break;
     }
     case ColorImage::Codec::YUV420p:{
         cv::Mat img( static_cast<int>(1.5*xvColorImage.height), xvColorImage.width, CV_8UC1, (void*)xvColorImage.data.get() );
         cv::cvtColor( img, cvMat, cv::COLOR_YUV420p2RGB );
         break;
     }
     case ColorImage::Codec::JPEG:{
         cv::Mat img( xvColorImage.height, xvColorImage.width, CV_8UC3, (void*)xvColorImage.data.get() );
         img = cv::imdecode( img, cv::IMREAD_COLOR );
         cv::cvtColor( img, cvMat, cv::COLOR_BGR2RGB );
         break;
     }
     case ColorImage::Codec::NV12:{
         cv::Mat img = cv::Mat( xvColorImage.height * 3/2, xvColorImage.width, CV_8UC1, (void*)xvColorImage.data.get() );
#if (CV_VERSION_MAJOR >= 4)
         cv::cvtColor(img, cvMat, cv::COLOR_YUV2RGB_YV12);
#else
         cv::cvtColor(img, cvMat, CV_YUV2RGB_YV12);
#endif
         break;
     }
     default: {
         /// TODO support BITSTREAM type
         std::cerr << "XVSDK-ROS-WRAPPER Error: unsupported color image type" << std::endl;
     }
  }

  return cvMat;
}

sensor_msgs::Image toRosImage(const ColorImage& xvColorImage, const std::string& frame_id)
{
  if (xvColorImage.hostTimestamp < 0)
    std::cerr << "XVSDK-ROS-WRAPPER toRosImage() Error: negative ColorImage host-timestamp" << std::endl;
    
  sensor_msgs::Image rosImage;
  rosImage.header.stamp.fromSec(xvColorImage.hostTimestamp > 0.1 ? xvColorImage.hostTimestamp : 0.1);
  rosImage.header.frame_id = frame_id;
  rosImage.height = xvColorImage.height;
  rosImage.width = xvColorImage.width;
  rosImage.encoding = sensor_msgs::image_encodings::BGR8; /// FIXME why need BGR to get RGB ?
  rosImage.is_bigendian = false;
  rosImage.step = xvColorImage.width * 3*sizeof(uint8_t); /// bytes for 1 line
  /// TODO How to avoid copy ?
  const int nbPx = xvColorImage.width * xvColorImage.height;
  std::vector<uint8_t> copy(3*nbPx);
  cv::Mat cvMat = toCvMatRGB(xvColorImage);
  std::memcpy(&copy[0], cvMat.data, nbPx * 3*sizeof(uint8_t));
  /// TODO use that instead when implemented
  //std::memcpy(&copy[0], xvColorImage.toRgb().data.get(), nbPx * 3*sizeof(uint8_t));
  rosImage.data = std::move(copy);

  return rosImage;
}

sensor_msgs::Image toRosImage(const DepthImage& xvDepthImage, const std::string& frame_id)
{
  if (xvDepthImage.hostTimestamp < 0)
    std::cerr << "XVSDK-ROS-WRAPPER toRosImage() Error: negative DepthImage host-timestamp" << std::endl;

  /// see https://www.ros.org/reps/rep-0117.html
  /// see https://www.ros.org/reps/rep-0118.html
  /// Detections that are too close to the sensor to quantify shall be represented by -Inf.
  /// Erroneous detections shall be represented by quiet (non-signaling) NaNs. Finally, out of range
  /// detections will be represented by +Inf.
  sensor_msgs::Image rosImage;
  rosImage.header.stamp.fromSec(xvDepthImage.hostTimestamp > 0.1 ? xvDepthImage.hostTimestamp : 0.1);
  rosImage.header.frame_id = frame_id;
  rosImage.height = xvDepthImage.height;
  rosImage.width = xvDepthImage.width;
  rosImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  rosImage.is_bigendian = false;
  rosImage.step = xvDepthImage.width * sizeof(float); /// bytes for 1 line
  /// TODO How to avoid copy ?
  const int nbPx = xvDepthImage.width * xvDepthImage.height;
  std::vector<uint8_t> copy(nbPx*sizeof(float)/sizeof(uint8_t));
  if (xvDepthImage.type == xv::DepthImage::Type::Depth_32) {
      std::memcpy(&copy[0], xvDepthImage.data.get(), nbPx * sizeof(float));
  } else if (xvDepthImage.type == xv::DepthImage::Type::Depth_16) {
      static float cov = 7.5 / 2494.0; // XXX *0.001
      const short* d = reinterpret_cast<const short*>(xvDepthImage.data.get());
      float* fl = reinterpret_cast<float*>(&copy[0]);
      for (int i = 0; i < nbPx; i++) {
          fl[i] = cov * d[i];
      }
  }
  rosImage.data = std::move(copy);

  float* depth = reinterpret_cast<float*>(&rosImage.data[0]);
  const float* const depth_end = reinterpret_cast<float*>(&*rosImage.data.end());
  for (; depth < depth_end; ++depth)
  {
    if (*depth < .2f)
      *depth = -std::numeric_limits<float>::infinity();
    else if (*depth > 10.f)
      *depth = std::numeric_limits<float>::infinity();
  }

  return rosImage;
}

static std::vector<std::vector<unsigned char>> colors = {{0,0,0},
{255,4,0}, {255,8,0}, {255,12,0}, {255,17,0}, {255,21,0}, {255,25,0}, {255,29,0},
{255,34,0}, {255,38,0}, {255,42,0}, {255,46,0}, {255,51,0}, {255,55,0}, {255,59,0},
{255,64,0}, {255,68,0}, {255,72,0}, {255,76,0}, {255,81,0}, {255,85,0}, {255,89,0},
{255,93,0}, {255,98,0}, {255,102,0}, {255,106,0}, {255,110,0}, {255,115,0}, {255,119,0},
{255,123,0}, {255,128,0}, {255,132,0}, {255,136,0}, {255,140,0}, {255,145,0}, {255,149,0},
{255,153,0}, {255,157,0}, {255,162,0}, {255,166,0}, {255,170,0}, {255,174,0}, {255,179,0},
{255,183,0}, {255,187,0}, {255,191,0}, {255,196,0}, {255,200,0}, {255,204,0}, {255,209,0},
{255,213,0}, {255,217,0}, {255,221,0}, {255,226,0}, {255,230,0}, {255,234,0}, {255,238,0},
{255,243,0}, {255,247,0}, {255,251,0}, {255,255,0}, {251,255,0}, {247,255,0}, {243,255,0},
{238,255,0}, {234,255,0}, {230,255,0}, {226,255,0}, {221,255,0}, {217,255,0}, {213,255,0},
{209,255,0}, {204,255,0}, {200,255,0}, {196,255,0}, {191,255,0}, {187,255,0}, {183,255,0},
{179,255,0}, {174,255,0}, {170,255,0}, {166,255,0}, {162,255,0}, {157,255,0}, {153,255,0},
{149,255,0}, {145,255,0}, {140,255,0}, {136,255,0}, {132,255,0}, {128,255,0}, {123,255,0},
{119,255,0}, {115,255,0}, {110,255,0}, {106,255,0}, {102,255,0}, {98,255,0}, {93,255,0},
{89,255,0}, {85,255,0}, {81,255,0}, {76,255,0}, {72,255,0}, {68,255,0}, {64,255,0},
{59,255,0}, {55,255,0}, {51,255,0}, {46,255,0}, {42,255,0}, {38,255,0}, {34,255,0},
{29,255,0}, {25,255,0}, {21,255,0}, {17,255,0}, {12,255,0}, {8,255,0}, {4,255,0},
{0,255,0}, {0,255,4}, {0,255,8}, {0,255,12}, {0,255,17}, {0,255,21}, {0,255,25},
{0,255,29}, {0,255,34}, {0,255,38}, {0,255,42}, {0,255,46}, {0,255,51}, {0,255,55},
{0,255,59}, {0,255,64}, {0,255,68}, {0,255,72}, {0,255,76}, {0,255,81}, {0,255,85},
{0,255,89}, {0,255,93}, {0,255,98}, {0,255,102}, {0,255,106}, {0,255,110}, {0,255,115},
{0,255,119}, {0,255,123}, {0,255,128}, {0,255,132}, {0,255,136}, {0,255,140}, {0,255,145},
{0,255,149}, {0,255,153}, {0,255,157}, {0,255,162}, {0,255,166}, {0,255,170}, {0,255,174},
{0,255,179}, {0,255,183}, {0,255,187}, {0,255,191}, {0,255,196}, {0,255,200}, {0,255,204},
{0,255,209}, {0,255,213}, {0,255,217}, {0,255,221}, {0,255,226}, {0,255,230}, {0,255,234},
{0,255,238}, {0,255,243}, {0,255,247}, {0,255,251}, {0,255,255}, {0,251,255}, {0,247,255},
{0,243,255}, {0,238,255}, {0,234,255}, {0,230,255}, {0,226,255}, {0,221,255}, {0,217,255},
{0,213,255}, {0,209,255}, {0,204,255}, {0,200,255}, {0,196,255}, {0,191,255}, {0,187,255},
{0,183,255}, {0,179,255}, {0,174,255}, {0,170,255}, {0,166,255}, {0,162,255}, {0,157,255},
{0,153,255}, {0,149,255}, {0,145,255}, {0,140,255}, {0,136,255}, {0,132,255}, {0,128,255},
{0,123,255}, {0,119,255}, {0,115,255}, {0,110,255}, {0,106,255}, {0,102,255}, {0,98,255},
{0,93,255}, {0,89,255}, {0,85,255}, {0,81,255}, {0,76,255}, {0,72,255}, {0,68,255},
{0,64,255}, {0,59,255}, {0,55,255}, {0,51,255}, {0,46,255}, {0,42,255}, {0,38,255},
{0,34,255}, {0,29,255}, {0,25,255}, {0,21,255}, {0,17,255}, {0,12,255}, {0,8,255},
{0,4,255}, {0,0,255}, {4,0,255}, {8,0,255}, {12,0,255}, {17,0,255}, {21,0,255},
{25,0,255}, {29,0,255}, {34,0,255}, {38,0,255}, {42,0,255}, {46,0,255}, {51,0,255},
{55,0,255}, {59,0,255}, {64,0,255}};

cv::Mat toCvMatRGB(const DepthColorImage& rgbd)
{
    cv::Mat out;
    out = cv::Mat::zeros(rgbd.height, rgbd.width*2, CV_8UC3);
    auto w = rgbd.width;

    if (rgbd.height>0 && rgbd.width>0) {
        float dmax = 7.5;
        const auto tmp_d = reinterpret_cast<std::uint8_t const*>(rgbd.data.get()+3);
        for (unsigned int i=0; i< rgbd.height*rgbd.width; i++) {
            const auto &d = *reinterpret_cast<float const*>(tmp_d + i*(3+sizeof(float)));
            if( d < 0.01 || d > 9.9 ) {
                out.at<cv::Vec3b>(i / w, i % rgbd.width) = 0;
            } else {
                unsigned int u = static_cast<unsigned int>( std::max(0.0f, std::min(255.0f,  d * 255.0f / dmax )));
                const auto &cc = colors.at(u);
                out.at<cv::Vec3b>( i/ w, i%rgbd.width ) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0) );
            }
        }
        const auto tmp_rgb = reinterpret_cast<std::uint8_t const*>(rgbd.data.get());
        for (unsigned int i=0; i< rgbd.height*rgbd.width; i++) {
            const auto rgb = reinterpret_cast<std::uint8_t const*>(tmp_rgb + i*(3+sizeof(float)));
            out.at<cv::Vec3b>( i/ w, (i%rgbd.width) + rgbd.width) = cv::Vec3b(rgb[0], rgb[1], rgb[2]);
        }
    }
    return out;
}

sensor_msgs::Image toRosImage(const DepthColorImage& xvDepthColorImage, const std::string& frame_id)
{
  if (xvDepthColorImage.hostTimestamp < 0)
    std::cerr << "XVSDK-ROS-WRAPPER toRosImage() Error: negative DepthColorImage host-timestamp" << std::endl;

  /// see https://www.ros.org/reps/rep-0117.html
  /// see https://www.ros.org/reps/rep-0118.html
  /// Detections that are too close to the sensor to quantify shall be represented by -Inf.
  /// Erroneous detections shall be represented by quiet (non-signaling) NaNs. Finally, out of range
  /// detections will be represented by +Inf.
  sensor_msgs::Image rosImage;
  rosImage.header.stamp.fromSec(xvDepthColorImage.hostTimestamp > 0.1 ? xvDepthColorImage.hostTimestamp : 0.1);
  rosImage.header.frame_id = frame_id;
  rosImage.height = xvDepthColorImage.height;
  rosImage.width = xvDepthColorImage.width * 2;
  rosImage.encoding = sensor_msgs::image_encodings::BGR8; /// FIXME why need BGR to get RGB ?
  rosImage.is_bigendian = false;
  rosImage.step = rosImage.width * 3*sizeof(uint8_t); /// bytes for 1 line
  const int nbPx = rosImage.width * rosImage.height;
  std::vector<uint8_t> copy(3*nbPx);
  cv::Mat cvMat = toCvMatRGB(xvDepthColorImage);
  std::memcpy(&copy[0], cvMat.data, nbPx * 3*sizeof(uint8_t));
  /// TODO use that instead when implemented
  //std::memcpy(&copy[0], xvColorImage.toRgb().data.get(), nbPx * 3*sizeof(uint8_t));
  rosImage.data = std::move(copy);

  return rosImage;
}

xv_sdk::Planes toRosPlanes(std::shared_ptr<const std::vector<Plane>> xvPlanes, const std::string& frame_id)
{
  xv_sdk::Planes msg;

  for (const auto& xvPlane : *xvPlanes)
  {
    xv_sdk::Plane plane;
    plane.frame_id = frame_id;

    for (const auto& vec3 : xvPlane.points)
      plane.points.emplace_back(toRosPoint(vec3));

    plane.normal = toRosVector3(xvPlane.normal);
    plane.d = xvPlane.d;
    plane.id = xvPlane.id;

    msg.planes.emplace_back(plane);
  }

  return msg;
}

sensor_msgs::CameraInfo toRosCameraInfo(const UnifiedCameraModel* const ucm, const PolynomialDistortionCameraModel* const pdcm)
{
  sensor_msgs::CameraInfo camInfo;
  if (pdcm)
  {
    /// Most ROS users will prefer PDM if available
    const auto& c = *pdcm;
    camInfo.height = c.h;
    camInfo.width = c.w;
    camInfo.distortion_model = "plumb_bob"; /// XXX is that correct ?
    camInfo.D = {c.distor.begin(), c.distor.end()};
    camInfo.K = {c.fx,    0, c.u0,
                    0, c.fy, c.v0,
                    0,    0,    1};
    //camInfo.R;
    //camInfo.P;
    camInfo.binning_x = 1;
    camInfo.binning_y = 1;
    //camInfo.roi;
  }
  else if (ucm)
  {
    const auto& c = *ucm;
    camInfo.height = c.h;
    camInfo.width = c.w;
    camInfo.distortion_model = "unified"; /// XXX is that correct ?
    camInfo.D = {c.xi};
    camInfo.K = {c.fx,    0, c.u0,
                    0, c.fy, c.v0,
                    0,    0,    1};
    //camInfo.R;
    //camInfo.P;
    camInfo.binning_x = 1;
    camInfo.binning_y = 1;
    //camInfo.roi;
  }

  return camInfo;
}

void computePointCloud(const DepthImage& xvDepthImage,
                       const ColorImage& xvColorImage,
                       /*const*/ Slam& xvSlam,
                       const Calibration& tof_calibration,
                       const Calibration& rgb_calibration,
                       cv::Mat& color_for_tof,
                       std::vector<Vector3d>& p3ds,
                       bool p3dsInWorldFrame = true)
{
  color_for_tof.create(xvDepthImage.height, xvDepthImage.width, CV_8UC3);
  color_for_tof = cv::Vec3b(0,0,0);

  Pose pose_at_tof_time, pose_at_rgb_time;
  if (!xvSlam.getPoseAt(pose_at_tof_time, xvDepthImage.hostTimestamp) ||
      !xvSlam.getPoseAt(pose_at_rgb_time, xvColorImage.hostTimestamp))
  {
    pose_at_tof_time.setTranslation({0,0,0});
    pose_at_tof_time.setRotation({1,0,0, 0,1,0, 0,0,1});
    pose_at_rgb_time.setTranslation({0,0,0});
    pose_at_rgb_time.setRotation({1,0,0, 0,1,0, 0,0,1});
  }

  cv::Mat rgb_mat;
  if (xvColorImage.data)
    rgb_mat = toCvMatRGB(xvColorImage);
  else
    rgb_mat = cv::Vec<uint8_t,3>(0,0,0);

  auto tof_pose_in_world = pose_at_tof_time * tof_calibration.pose;

  xv::PolynomialDistortionCameraModel tof_intrinsic;
  for (auto& cal : tof_calibration.pdcm){
    if (cal.h == (int)xvDepthImage.height){
      tof_intrinsic = cal;
      break;
    }
  }
  xv::PolynomialDistortionCameraModel rgb_intrinsic;
  for (auto& cal : rgb_calibration.pdcm){
    if (cal.h == rgb_mat.rows){
      rgb_intrinsic = cal;
      break;
    }
  }

  float const* tof_ptr = reinterpret_cast<float const*>(xvDepthImage.data.get());

  p3ds.reserve(xvDepthImage.height*xvDepthImage.width);

  for (int j = 0; j < int(xvDepthImage.height); ++j)
  for (int i = 0; i < int(xvDepthImage.width); ++i)
  {
    float tmp_depth = 0.0;
    if(xvDepthImage.type == xv::DepthImage::Type::Depth_16)
    {
      tmp_depth = reinterpret_cast<short const*>(xvDepthImage.data.get())[i+j*xvDepthImage.width] / 1000.0;
    }
    else
    {
      tmp_depth = tof_ptr[i+j*xvDepthImage.width];
    }
    
    const float depth = tmp_depth;

    Vector2d p2d_tof{double(i),double(j)};

    Vector3d ray_in_tof;

    if (depth > .2f && depth < 5.f && raytrace(tof_intrinsic, p2d_tof, ray_in_tof))
    {
      Vector3d p3d_in_tof = ray_in_tof;
      for (double& x : p3d_in_tof)
        x *= depth;

      const Vector3d p3d_in_world = Transform(tof_pose_in_world.translation(), tof_pose_in_world.rotation()) * p3d_in_tof;

      if (p3dsInWorldFrame)
        p3ds.emplace_back(p3d_in_world);
      else
        p3ds.emplace_back(p3d_in_tof);

      Vector2d p2d_in_rgb;
      const auto rgb_in_world = Transform(pose_at_rgb_time.translation(), pose_at_rgb_time.rotation());

      if (project(rgb_in_world, rgb_calibration.pose, rgb_intrinsic, p3d_in_world, p2d_in_rgb))
      {
        if (p2d_in_rgb[0] >= 0 && p2d_in_rgb[0] < rgb_intrinsic.w && p2d_in_rgb[1] >=0 && p2d_in_rgb[1] < rgb_intrinsic.h)
        {
          const cv::Vec3b rgb = rgb_mat.at<cv::Vec3b>(p2d_in_rgb[1],p2d_in_rgb[0]);
          color_for_tof.at<cv::Vec3b>(p2d_tof[1],p2d_tof[0]) = rgb;
        }
      }
    }
    else {
      p3ds.emplace_back(Vector3d{std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN()});
    }
  }
}

void computePointCloud(const SgbmImage& xvSgbmImage,
                       const ColorImage& xvColorImage,
                       /*const*/ Slam& xvSlam,
                       const Calibration& sgbm_calibration,
                       const Calibration& rgb_calibration,
                       cv::Mat& color_for_sgbm,
                       std::vector<Vector3d>& p3ds,
                       bool p3dsInWorldFrame = true)
{
  color_for_sgbm.create(xvSgbmImage.height, xvSgbmImage.width, CV_8UC3);
  color_for_sgbm = cv::Vec3b(0,0,0);

  Pose pose_at_sgbm_time, pose_at_rgb_time;
  if (!xvSlam.getPoseAt(pose_at_sgbm_time, xvSgbmImage.hostTimestamp) ||
      !xvSlam.getPoseAt(pose_at_rgb_time, xvColorImage.hostTimestamp))
  {
    pose_at_sgbm_time.setTranslation({0,0,0});
    pose_at_sgbm_time.setRotation({1,0,0, 0,1,0, 0,0,1});
    pose_at_rgb_time.setTranslation({0,0,0});
    pose_at_rgb_time.setRotation({1,0,0, 0,1,0, 0,0,1});
  }

  cv::Mat rgb_mat;
  if (xvColorImage.data)
    rgb_mat = toCvMatRGB(xvColorImage);
  else
  {
    rgb_mat = cv::Vec<uint8_t,3>(0,0,0);
  }

  auto sgbm_pose_in_world = pose_at_sgbm_time * sgbm_calibration.pose;

  xv::UnifiedCameraModel sgbm_intrinsic;
  for (auto& cal : sgbm_calibration.ucm){
    if (cal.h == (int)xvSgbmImage.height){
      
      sgbm_intrinsic = cal;
      break;
    }
  }
  xv::PolynomialDistortionCameraModel rgb_intrinsic;
  for (auto& cal : rgb_calibration.pdcm){
    if (cal.h == rgb_mat.rows){
      rgb_intrinsic = cal;
      break;
    }
  }

  //float const* sgbm_ptr = reinterpret_cast<float const*>(xvSgbmImage.data.get());

  p3ds.reserve(xvSgbmImage.height*xvSgbmImage.width);

  for (int j = 0; j < int(xvSgbmImage.height); ++j)
  for (int i = 0; i < int(xvSgbmImage.width); ++i)
  {
    float tmp_depth =  reinterpret_cast<short const*>(xvSgbmImage.data.get())[i+j*xvSgbmImage.width] / 1000.0;
    const float depth = tmp_depth;
    Vector2d p2d_sgbm{double(i),double(j)};

    Vector3d ray_in_sgbm;
    if (depth > .2f && depth < 5.f && raytrace(sgbm_intrinsic, p2d_sgbm, ray_in_sgbm))
    {
      Vector3d p3d_in_sgbm = ray_in_sgbm;
      for (double& x : p3d_in_sgbm)
        x *= depth;

      const Vector3d p3d_in_world = Transform(sgbm_pose_in_world.translation(), sgbm_pose_in_world.rotation()) * p3d_in_sgbm;

      if (p3dsInWorldFrame)
        p3ds.emplace_back(p3d_in_world);
      else
        p3ds.emplace_back(p3d_in_sgbm);

      Vector2d p2d_in_rgb;
      const auto rgb_in_world = Transform(pose_at_rgb_time.translation(), pose_at_rgb_time.rotation());

      if (project(rgb_in_world, rgb_calibration.pose, rgb_intrinsic, p3d_in_world, p2d_in_rgb))
      {
        if (p2d_in_rgb[0] >= 0 && p2d_in_rgb[0] < rgb_intrinsic.w && p2d_in_rgb[1] >=0 && p2d_in_rgb[1] < rgb_intrinsic.h)
        {
          const cv::Vec3b rgb = rgb_mat.at<cv::Vec3b>(p2d_in_rgb[1],p2d_in_rgb[0]);
          color_for_sgbm.at<cv::Vec3b>(p2d_sgbm[1],p2d_sgbm[0]) = rgb;
        }
      }
    }
    else {
      p3ds.emplace_back(Vector3d{std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN()});
    }
  }
}

void computePointCloud(SgbmImage const &sgbmImage,
                       /*const*/ Slam& xvSlam,
                       std::vector<Vector3d>& p3ds,
                       pointCloudUnit pUnit,
                       bool p3dsInWorldFrame = true)
{
    if(sgbmImage.type != xv::SgbmImage::Type::PointCloud)
    {
        return;
    }
    Pose pose_at_pc_time;
    if (!xvSlam.getPoseAt(pose_at_pc_time, sgbmImage.hostTimestamp))
    {
        pose_at_pc_time.setTranslation({0,0,0});
        pose_at_pc_time.setRotation({1,0,0, 0,1,0, 0,0,1});
    }
    auto sgbm_pose_in_world = pose_at_pc_time ;//* pc_calibration.pose;

    short *dataTemp = (short *)sgbmImage.data.get();

    unsigned int size = sgbmImage.width * sgbmImage.height;
    for (unsigned int i=0; i < size; i++)
  {
      Vector3d p3d_in_sgbm = { (pUnit == pointCloudUnit::m) ?(double((*dataTemp)/1000.0)):(double(*dataTemp)), 
                               (pUnit == pointCloudUnit::m) ?(double((*(dataTemp + 1))/1000.0)):(double(*(dataTemp + 1))), 
                               (pUnit == pointCloudUnit::m) ?(double((*(dataTemp + 2))/1000.0)):(double(*(dataTemp + 2)))};
      const Vector3d p3d_in_world = Transform(sgbm_pose_in_world.translation(), sgbm_pose_in_world.rotation()) * p3d_in_sgbm;

      if (p3dsInWorldFrame)
        p3ds.emplace_back(p3d_in_world);
      else
        p3ds.emplace_back(p3d_in_sgbm);
      dataTemp += 3;
  }
  // std::cout << "i:"<< i<< std::endl;
}

void computePointCloud(std::shared_ptr<PointCloud> xvPointCloud,
                       /*const*/ Slam& xvSlam,
                       std::vector<Vector3d>& p3ds,
                       pointCloudUnit pUnit,
                       bool p3dsInWorldFrame = true)
{
  Pose pose_at_pc_time;
  if (!xvSlam.getPoseAt(pose_at_pc_time, xvPointCloud->hostTimestamp))
  {
    pose_at_pc_time.setTranslation({0,0,0});
    pose_at_pc_time.setRotation({1,0,0, 0,1,0, 0,0,1});
  }

  auto sgbm_pose_in_world = pose_at_pc_time ;//* pc_calibration.pose;

  p3ds.reserve(xvPointCloud->points.size());

  for (int i = 0; i < int(xvPointCloud->points.size()); ++i)
  {
      Vector3d p3d_in_sgbm = { (pUnit == pointCloudUnit::m) ?(double(xvPointCloud->points[i][0]/1000.0)):(double(xvPointCloud->points[i][0])), 
                               (pUnit == pointCloudUnit::m) ?(double(xvPointCloud->points[i][1]/1000.0)):(double(xvPointCloud->points[i][1])), 
                               (pUnit == pointCloudUnit::m) ?(double(xvPointCloud->points[i][2]/1000.0)):(double(xvPointCloud->points[i][2]))};
      const Vector3d p3d_in_world = Transform(sgbm_pose_in_world.translation(), sgbm_pose_in_world.rotation()) * p3d_in_sgbm;

      if (p3dsInWorldFrame)
        p3ds.emplace_back(p3d_in_world);
      else
        p3ds.emplace_back(p3d_in_sgbm);
  
  }
}

void computePointCloud(const SgbmImage& xvSgbmImage,
                       /*const*/ Slam& xvSlam,
                       const Calibration& sgbm_calibration,
                       std::vector<Vector3d>& p3ds,
                       bool p3dsInWorldFrame = true)
{
  Pose pose_at_sgbm_time;
  if (!xvSlam.getPoseAt(pose_at_sgbm_time, xvSgbmImage.hostTimestamp))
  {
    pose_at_sgbm_time.setTranslation({0,0,0});
    pose_at_sgbm_time.setRotation({1,0,0, 0,1,0, 0,0,1});
  }

  auto sgbm_pose_in_world = pose_at_sgbm_time * sgbm_calibration.pose;

  xv::UnifiedCameraModel sgbm_intrinsic;
  for (auto& cal : sgbm_calibration.ucm){
    if (cal.h == (int)xvSgbmImage.height){
      
      sgbm_intrinsic = cal;
      break;
    }
  }

  //float const* sgbm_ptr = reinterpret_cast<float const*>(xvSgbmImage.data.get());

  p3ds.reserve(xvSgbmImage.height*xvSgbmImage.width);

  for (int j = 0; j < int(xvSgbmImage.height); ++j)
  for (int i = 0; i < int(xvSgbmImage.width); ++i)
  {
    float tmp_depth =  reinterpret_cast<short const*>(xvSgbmImage.data.get())[i+j*xvSgbmImage.width] / 1000.0;
    const float depth = tmp_depth;
    Vector2d p2d_sgbm{double(i),double(j)};

    Vector3d ray_in_sgbm;
    if (depth > .2f && depth < 5.f && raytrace(sgbm_intrinsic, p2d_sgbm, ray_in_sgbm))
    {
      Vector3d p3d_in_sgbm = ray_in_sgbm;
      for (double& x : p3d_in_sgbm)
        x *= depth;

      const Vector3d p3d_in_world = Transform(sgbm_pose_in_world.translation(), sgbm_pose_in_world.rotation()) * p3d_in_sgbm;

      if (p3dsInWorldFrame)
        p3ds.emplace_back(p3d_in_world);
      else
        p3ds.emplace_back(p3d_in_sgbm);
    }
    else {
      p3ds.emplace_back(Vector3d{std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN()});
    }
  }
}

sensor_msgs::Image toRosImage(cv::Mat cvColorImage, double timestamp, const std::string& frame_id)
{
  if (timestamp < 0)
    std::cerr << "XVSDK-ROS-WRAPPER toRosImage() Error: negative cvColorImage host-timestamp" << std::endl;

  sensor_msgs::Image rosImage;
  rosImage.header.stamp.fromSec(timestamp > 0.1 ? timestamp : 0.1);
  rosImage.header.frame_id = frame_id;
  rosImage.height = cvColorImage.rows;
  rosImage.width = cvColorImage.cols;
  rosImage.encoding = sensor_msgs::image_encodings::RGB8;
  rosImage.is_bigendian = false;
  rosImage.step = cvColorImage.cols * 3*sizeof(uint8_t); /// bytes for 1 line
  /// TODO How to avoid copy ?
  const int nbPx = cvColorImage.cols * cvColorImage.rows;
  std::vector<uint8_t> copy(3*nbPx);
  std::memcpy(&copy[0], cvColorImage.data, nbPx * 3*sizeof(uint8_t));
  rosImage.data = std::move(copy);

  return rosImage;
}

static std::tuple<int, int, int> color(double distance, double distance_min, double distance_max, double threshold)
{
    double d = std::max(distance_min, std::min(distance, distance_max));
    d = (d - distance_min) / (distance_max - distance_min);
    // std::cout<<"color max"<<distance_max<<"color min"<<distance_min<<"color"<<distance<<std::endl;
    if (distance <= threshold || distance > distance_max)
    {
        return std::tuple<int, int, int>(0, 0, 0);
    }
    int b = static_cast<int>(255.0 * std::min(std::max(0.0, 1.5 - std::abs(1.0 - 4.0 * (d - 0.5))), 1.0));
    int g = static_cast<int>(255.0 * std::min(std::max(0.0, 1.5 - std::abs(1.0 - 4.0 * (d - 0.25))), 1.0));
    int r = static_cast<int>(255.0 * std::min(std::max(0.0, 1.5 - std::abs(1.0 - 4.0 * d)), 1.0));
    return std::tuple<int, int, int>(r, g, b);
}

static std::shared_ptr<unsigned char> depthImage(uint16_t *data, unsigned int width, unsigned int height, double min_distance_m, double max_distance_m, bool colorize)
{
    std::shared_ptr<unsigned char> out;
    if (colorize)
    {
        out = std::shared_ptr<unsigned char>(new unsigned char[width * height * 3], std::default_delete<unsigned char[]>());
    }
    else
    {
        out = std::shared_ptr<unsigned char>(new unsigned char[width * height], std::default_delete<unsigned char[]>());
    }

    for (unsigned int i = 0; i < width * height; i++)
    {
        double distance_mm = data[i];
        if (colorize)
        {
            double distance_m = distance_mm / 1000.;

            auto c = color(distance_m, min_distance_m, max_distance_m, min_distance_m);
            out.get()[i * 3 + 0] = static_cast<unsigned char>(std::get<2>(c));
            out.get()[i * 3 + 1] = static_cast<unsigned char>(std::get<1>(c));
            out.get()[i * 3 + 2] = static_cast<unsigned char>(std::get<0>(c));
        }
        else
        {
            double max_distance_mm = max_distance_m * 1000.;
            double min_distance_mm = min_distance_m * 1000.;
            distance_mm = std::min(max_distance_mm, distance_mm);
            distance_mm = std::max(distance_mm, min_distance_mm);

            double norm = (distance_mm - min_distance_mm) / (max_distance_mm - min_distance_mm);
            auto c = 255. * norm;
            out.get()[i] = static_cast<unsigned char>(c);
        }
    }

    return out;
}
cv::Mat convDepthToMat(std::shared_ptr<const xv::SgbmImage> sgbm_image,bool _colorize_depth)
{
    static double depth_max_distance_m = 5;
    static double depth_min_distance_m = 0.1;
    uint16_t* p16 = (uint16_t*)sgbm_image->data.get();

    // cv::Mat mask;
    // cv::Mat im_gray_d = cv::Mat(cv::Size(sgbm_image->width, sgbm_image->height),  CV_16UC1, p16); //18
    // cv::inRange(im_gray_d, cv::Scalar(1), cv::Scalar(65535), mask);
    // p16 = (uint16_t *)im_gray_d.data;

    double focal_length = sgbm_image->width / (2.f * tan(/*global_config.fov*/69 / 2 / 180.f * M_PI));
    double max_distance_m = (focal_length * /*global_config.baseline*/0.11285 / 1);
    double min_distance_m = 0; //0 is considered invalid distance (0 disparity == unknown)
    max_distance_m = std::min(max_distance_m, depth_max_distance_m);
    min_distance_m = depth_min_distance_m;
    assert(max_distance_m > min_distance_m);

    static std::shared_ptr<unsigned char> tmp;
    tmp = depthImage(p16, sgbm_image->width, sgbm_image->height, min_distance_m, max_distance_m, !!_colorize_depth);
    if (_colorize_depth)
    {
        cv::Mat im_col(cv::Size(sgbm_image->width, sgbm_image->height), CV_8UC3, tmp.get());
        // cv::Mat roi = cv::Mat::zeros(cv::Size(sgbm_image->width, sgbm_image->height), CV_8UC3);
        // im_col.copyTo(roi,mask);
        return im_col;
    }
    else
    {
        cv::Mat im_col(cv::Size(sgbm_image->width, sgbm_image->height), CV_8UC1, tmp.get());
        return im_col;
    }
}

sensor_msgs::Image toRosImage(const SgbmImage& xvSgbmDepthImage, const std::string& frame_id)
{
  if (xvSgbmDepthImage.hostTimestamp < 0)
    std::cerr << "XVSDK-ROS-WRAPPER toRosImage() Error: negative SgbmImage host-timestamp" << std::endl;

  /// see https://www.ros.org/reps/rep-0117.html
  /// see https://www.ros.org/reps/rep-0118.html
  /// Detections that are too close to the sensor to quantify shall be represented by -Inf.
  /// Erroneous detections shall be represented by quiet (non-signaling) NaNs. Finally, out of range
  /// detections will be represented by +Inf.
  sensor_msgs::Image rosImage;

  // if(xv::SgbmImage::Type::Depth != xvSgbmDepthImage.type)
  // {
  //   std::cerr << "XVSDK-ROS-WRAPPER toRosImage() Error: wrong sgbm type:"<< std::endl;
  // }
  if(xv::SgbmImage::Type::Disparity == xvSgbmDepthImage.type)
  {
    std::cerr << "XVSDK-ROS-WRAPPER toRosImage()s Error: wrong sgbm type:Disparity"<< std::endl;
  }
  else
  {
    rosImage.header.stamp.fromSec(xvSgbmDepthImage.hostTimestamp > 0.1 ? xvSgbmDepthImage.hostTimestamp : 0.1);
    rosImage.header.frame_id = frame_id;
    
    rosImage.height = xvSgbmDepthImage.height;
     
    rosImage.width = xvSgbmDepthImage.width;
    
    rosImage.encoding = sensor_msgs::image_encodings::BGR8; /// FIXME why need BGR to get RGB ?
    rosImage.is_bigendian = false;
   
    rosImage.step = rosImage.width * 3*sizeof(uint8_t); /// bytes for 1 line
   
    const int nbPx = rosImage.width * rosImage.height;
     
    std::vector<uint8_t> copy(3*nbPx);
    std::shared_ptr<const xv::SgbmImage> ptr_sgbm = std::make_shared<xv::SgbmImage>(xvSgbmDepthImage);
    cv::Mat cvMat = convDepthToMat(std::shared_ptr<const xv::SgbmImage>(ptr_sgbm),true);
    std::memcpy(&copy[0], cvMat.data, nbPx * 3*sizeof(uint8_t));
    rosImage.data = std::move(copy);
  }

  return rosImage;
}

sensor_msgs::PointCloud2 toRosPointCloud2(cv::Mat cvColorImage,
                                          const std::vector<Vector3d>& p3ds_in_world,
                                          double timestamp,
                                          const std::string& frame_id)
{
  auto genFields = []()
  {
    std::vector<sensor_msgs::PointField> fds;
    sensor_msgs::PointField f;
    f.name = "x";
    f.offset = 0;
    f.datatype = sensor_msgs::PointField::FLOAT32;
    f.count = 1;
    fds.emplace_back(f);
    f.name = "y";
    f.offset = 4;
    f.datatype = sensor_msgs::PointField::FLOAT32;
    f.count = 1;
    fds.emplace_back(f);
    f.name = "z";
    f.offset = 8;
    f.datatype = sensor_msgs::PointField::FLOAT32;
    f.count = 1;
    fds.emplace_back(f);

    f.name = "rgba";
    f.offset = 12;
    f.datatype = sensor_msgs::PointField::UINT32;
    f.count = 1;
    fds.emplace_back(f);
    return fds;
  };
  static const std::vector<sensor_msgs::PointField> fields = genFields();
  struct __attribute__((packed)) F {float x,y,z; uint8_t r,g,b,a;};
  sensor_msgs::PointCloud2 pc2;
  pc2.header.stamp.fromSec(timestamp > 0.1 ? timestamp : 0.1);
  pc2.header.frame_id = frame_id;
  pc2.height = cvColorImage.rows;
  pc2.width = cvColorImage.cols;
  pc2.fields = fields; /// xyzrgb
  pc2.is_bigendian = false;
  pc2.point_step = 3*sizeof(float) + 4*sizeof(uint8_t);
  pc2.row_step = pc2.point_step * pc2.width;
  pc2.is_dense = false;
  const int nbPt = pc2.height * pc2.width;
  pc2.data.resize(nbPt*pc2.point_step);
  F* data = reinterpret_cast<F*>(pc2.data.data());
  const cv::Vec3b* rgb = &cvColorImage.at<cv::Vec3b>(0);
  const Vector3d* xyz = p3ds_in_world.data();
  const Vector3d* const xyz_end = &*p3ds_in_world.end();
  for (; xyz < xyz_end; ++rgb, ++xyz, ++data)
  {
    *data = F {
      float((*xyz)[0]), float((*xyz)[1]), float((*xyz)[2]),
      (*rgb)[0], (*rgb)[1], (*rgb)[2], 255
    };
  }
  return pc2;
}

sensor_msgs::PointCloud2 toRosPointCloud2(const std::vector<Vector3d>& p3ds_in_world,
                                          double timestamp,
                                          const std::string& frame_id)
{
  auto genFields = []()
  {
    std::vector<sensor_msgs::PointField> fds;
    sensor_msgs::PointField f;

    f.name = "x";
    f.offset = 0;
    f.datatype = sensor_msgs::PointField::FLOAT32;
    f.count = 1;
    fds.emplace_back(f);
    f.name = "y";
    f.offset = 4;
    f.datatype = sensor_msgs::PointField::FLOAT32;
    f.count = 1;
    fds.emplace_back(f);
    f.name = "z";
    f.offset = 8;
    f.datatype = sensor_msgs::PointField::FLOAT32;
    f.count = 1;
    fds.emplace_back(f);

    return fds;
  };
  static const std::vector<sensor_msgs::PointField> fields = genFields();
  struct __attribute__((packed)) F {float x,y,z;};
  sensor_msgs::PointCloud2 pc2;
  pc2.header.stamp.fromSec(steady_clock_now());
  pc2.header.frame_id = frame_id;
  pc2.height = 1;
  const int nbPt = p3ds_in_world.size();
  pc2.width = nbPt;
  pc2.fields = fields; /// xyz
  pc2.is_bigendian = false;
  pc2.point_step = 3*sizeof(float);
  pc2.row_step = pc2.point_step * pc2.width;
  pc2.is_dense = true;

  pc2.data.resize(nbPt*pc2.point_step);
  F* data = reinterpret_cast<F*>(pc2.data.data());

  const Vector3d* xyz = p3ds_in_world.data();
  const Vector3d* const xyz_end = &*p3ds_in_world.end();

  for (; xyz < xyz_end; ++xyz, ++data)
  {
    *data = F { float((*xyz)[0]), float((*xyz)[1]), float((*xyz)[2]) };
  }
  return pc2;
}

void toRos(const xv::SlamMap& xvSlamMap, sensor_msgs::PointCloud2& mapPoints, visualization_msgs::MarkerArray& keyFrames, const std::string& frame_id)
{
  auto genFields = []()
  {
    std::vector<sensor_msgs::PointField> fds;
    sensor_msgs::PointField f;

    f.name = "x";
    f.offset = 0;
    f.datatype = sensor_msgs::PointField::FLOAT32;
    f.count = 1;
    fds.emplace_back(f);
    f.name = "y";
    f.offset = 4;
    f.datatype = sensor_msgs::PointField::FLOAT32;
    f.count = 1;
    fds.emplace_back(f);
    f.name = "z";
    f.offset = 8;
    f.datatype = sensor_msgs::PointField::FLOAT32;
    f.count = 1;
    fds.emplace_back(f);

    return fds;
  };
  static const std::vector<sensor_msgs::PointField> fields = genFields();
  struct __attribute__((packed)) F {float x,y,z;};

  mapPoints.header.stamp.fromSec(steady_clock_now());
  mapPoints.header.frame_id = frame_id;
  mapPoints.height = 1;
  const int nbPt = xvSlamMap.vertices.size();
  mapPoints.width = nbPt;
  mapPoints.fields = fields; /// xyz
  mapPoints.is_bigendian = false;
  mapPoints.point_step = 3*sizeof(float);
  mapPoints.row_step = mapPoints.point_step * mapPoints.width;
  mapPoints.is_dense = true;

  mapPoints.data.resize(nbPt*mapPoints.point_step);
  F* data = reinterpret_cast<F*>(mapPoints.data.data());
  const Vector3d* xyz = xvSlamMap.vertices.data();
  const Vector3d* const xyz_end = &*xvSlamMap.vertices.end();
  for (; xyz < xyz_end; ++xyz, ++data)
  {
    *data = F { float((*xyz)[0]), float((*xyz)[1]), float((*xyz)[2]) };
  }

  /// Key frames
  /// XXX need xvSlamMap.poses
/*
  keyFrames.markers.reserve(xvSlamMap.poses.size());
  for (int i = 0; i < int(xvSlamMap.poses.size()); ++i)
  {
    const auto& pose = xvSlamMap.poses[i];

    visualization_msgs::Marker m;
    m.header.stamp = mapPoints.header.stamp;
    m.header.frame_id = frame_id;
    m.ns = "slam_key_frames";
    m.id = i;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::MODIFY;
    m.pose = toRosPose(pose);
    m.scale.x =
    m.scale.y =
    m.scale.z = .05;
    m.color.r = 0.;
    m.color.g = 1.;
    m.color.b = 0.;
    m.color.a = 1.;
    m.lifetime.fromSec(0); /// forever
    m.frame_locked = false; /// ?

    keyFrames.markers.emplace_back(m);
  }
*/
}

static std::map<std::string, int> s_stereoPlaneId_to_int;
visualization_msgs::MarkerArray stereoPlanesToRosMarkerArray(std::shared_ptr<const std::vector<Plane>> xvPlanes,
                                                             const std::string& frame_id)
{
  visualization_msgs::MarkerArray ma;
  ros::Time ts;
  ts.fromSec(steady_clock_now());

  ma.markers.reserve(xvPlanes->size());
  for (const auto& xvPlane : *xvPlanes)
  {
    visualization_msgs::Marker p;
    p.header.stamp = ts;
    p.header.frame_id = frame_id;
    p.ns = "stereo_planes";
    if (s_stereoPlaneId_to_int.find(xvPlane.id) == s_stereoPlaneId_to_int.end())
      s_stereoPlaneId_to_int[xvPlane.id] = s_stereoPlaneId_to_int.size();
    p.id = s_stereoPlaneId_to_int[xvPlane.id];
    p.type = visualization_msgs::Marker::LINE_STRIP;
    p.action = visualization_msgs::Marker::MODIFY;
    p.pose.orientation.x =
    p.pose.orientation.y =
    p.pose.orientation.z = 0.;
    p.pose.orientation.w = 1.;
    p.pose.position.x =
    p.pose.position.y =
    p.pose.position.z = 0.;
    p.scale.x = .01; /// line width
    p.color.r = 0.;
    p.color.g = 1.;
    p.color.b = 0.;
    p.color.a = .3;
    p.lifetime.fromSec(0); /// forever
    p.frame_locked = false; /// ?

    p.points.reserve(xvPlane.points.size()+1);
    for (const auto& pt : xvPlane.points)
      p.points.emplace_back(toRosPoint(pt));
    p.points.emplace_back(toRosPoint(xvPlane.points[0]));

    ma.markers.emplace_back(p);

    /// text

    visualization_msgs::Marker txt;
    txt.header.stamp = ts;
    txt.header.frame_id = frame_id;
    txt.ns = "stereo_planes_id";
    txt.id = s_stereoPlaneId_to_int[xvPlane.id];
    txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::Marker::MODIFY;
    txt.pose.orientation.x =
    txt.pose.orientation.y =
    txt.pose.orientation.z = 0.;
    txt.pose.orientation.w = 1.;
    txt.pose.position.x = xvPlane.points[0][0];
    txt.pose.position.y = xvPlane.points[0][1];
    txt.pose.position.z = xvPlane.points[0][2];
    txt.scale.z = .1; /// text height
    txt.color.r = 0.;
    txt.color.g = 1.;
    txt.color.b = 0.;
    txt.color.a = 1.;
    txt.lifetime.fromSec(0); /// forever
    txt.frame_locked = false; /// ?
    txt.text = "ID:"+xvPlane.id;

    ma.markers.emplace_back(txt);
  }

  return ma;
}

static std::map<std::string, int> s_tofPlaneId_to_int;
visualization_msgs::MarkerArray tofPlanesToRosMarkerArray(std::shared_ptr<const std::vector<Plane>> xvPlanes,
                                                          const std::string& frame_id)
{
  visualization_msgs::MarkerArray ma;
  ros::Time ts;
  ts.fromSec(steady_clock_now());

  ma.markers.reserve(xvPlanes->size());
  for (const auto& xvPlane : *xvPlanes)
  {
    visualization_msgs::Marker p;
    p.header.stamp = ts;
    p.header.frame_id = frame_id;
    p.ns = "tof_planes";
    if (s_tofPlaneId_to_int.find(xvPlane.id) == s_tofPlaneId_to_int.end())
      s_tofPlaneId_to_int[xvPlane.id] = s_tofPlaneId_to_int.size();
    p.id = s_tofPlaneId_to_int[xvPlane.id];
    p.type = visualization_msgs::Marker::LINE_STRIP;
    p.action = visualization_msgs::Marker::MODIFY;
    p.pose.orientation.x =
    p.pose.orientation.y =
    p.pose.orientation.z = 0.;
    p.pose.orientation.w = 1.;
    p.pose.position.x =
    p.pose.position.y =
    p.pose.position.z = 0.;
    p.scale.x = .01; /// line width
    p.color.r = 1.;
    p.color.g = .3;
    p.color.b = 0.;
    p.color.a = .3;
    p.lifetime.fromSec(0); /// forever
    p.frame_locked = false; /// ?

    p.points.reserve(xvPlane.points.size()+1);
    for (const auto& pt : xvPlane.points)
      p.points.emplace_back(toRosPoint(pt));
    p.points.emplace_back(toRosPoint(xvPlane.points[0]));

    ma.markers.emplace_back(p);

    /// text

    visualization_msgs::Marker txt;
    txt.header.stamp = ts;
    txt.header.frame_id = frame_id;
    txt.ns = "tof_planes_id";
    txt.id = s_tofPlaneId_to_int[xvPlane.id];
    txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::Marker::MODIFY;
    txt.pose.orientation.x =
    txt.pose.orientation.y =
    txt.pose.orientation.z = 0.;
    txt.pose.orientation.w = 1.;
    txt.pose.position.x = xvPlane.points[0][0];
    txt.pose.position.y = xvPlane.points[0][1];
    txt.pose.position.z = xvPlane.points[0][2];
    txt.scale.z = .1; /// text height
    txt.color.r = 1.;
    txt.color.g = .3;
    txt.color.b = 0.;
    txt.color.a = 1.;
    txt.lifetime.fromSec(0); /// forever
    txt.frame_locked = false; /// ?
    txt.text = "ID:"+xvPlane.id;

    ma.markers.emplace_back(txt);
  }

  return ma;
}

} /// namespace

RosDevice::RosDevice(ros::NodeHandle* parent, const std::string& rosNamespace, std::shared_ptr<Device> device)
  : m_nodeHandle(*parent, rosNamespace) /// each device has its own ROS namespace
  , m_rosNamespace(rosNamespace)
  , m_xvDevice(device)
  , m_ddynReconfig(new ddynamic_reconfigure::DDynamicReconfigure(m_nodeHandle))
  , m_nodeHandle_slam(m_nodeHandle, "slam")
  , m_ddynReconfig_slam(new ddynamic_reconfigure::DDynamicReconfigure(m_nodeHandle_slam))
  , m_nodeHandle_edge(m_nodeHandle, "edge")
  , m_ddynReconfig_edge(new ddynamic_reconfigure::DDynamicReconfigure(m_nodeHandle_edge))
  , m_nodeHandle_imuSensor(m_nodeHandle, "imu_sensor")
  , m_ddynReconfig_imuSensor(new ddynamic_reconfigure::DDynamicReconfigure(m_nodeHandle_imuSensor))
  , m_nodeHandle_fisheye(m_nodeHandle, "fisheye_cameras")
  , m_nodeHandle_fisheyeLeft(m_nodeHandle_fisheye, "left")
  , m_nodeHandle_fisheyeRight(m_nodeHandle_fisheye, "right")
  , m_ddynReconfig_fisheye(new ddynamic_reconfigure::DDynamicReconfigure(m_nodeHandle_fisheye))
  , m_nodeHandle_colorCamera(m_nodeHandle, "color_camera")
  , m_ddynReconfig_colorCamera(new ddynamic_reconfigure::DDynamicReconfigure(m_nodeHandle_colorCamera))
  , m_nodeHandle_sgbmCamera(m_nodeHandle, "sgbm_camera")
  , m_ddynReconfig_sgbmCamera(new ddynamic_reconfigure::DDynamicReconfigure(m_nodeHandle_sgbmCamera))
  , m_nodeHandle_tofCamera(m_nodeHandle, "tof_camera")
  , m_ddynReconfig_tofCamera(new ddynamic_reconfigure::DDynamicReconfigure(m_nodeHandle_tofCamera))
{
    initPublisherAdvertise();
    init();
}

std::string RosDevice::getRosNamespace() const
{
  return m_rosNamespace;
}

std::string RosDevice::getFrameId(const std::string& defaultId) const
{
  GUARD_PARAMS;
  return m_param_tfPrefix+"/"+s_frameIds.at(defaultId);
}

std::string timeShowStr(std::int64_t edgeTimestampUs, double hostTimestamp) {
    char s[1024];
    double now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()*1e-6;
    std::sprintf(s, " (device=%lld host=%.4f now=%.4f delay=%.4f) ", (long long)edgeTimestampUs, hostTimestamp, now, now-hostTimestamp);
    return std::string(s);
}
void showSgbmDepthImage(const RosDevice* device, const SgbmImage & xvSgbmImage);
void showSgbmPointCloudImage(RosDevice* device, const SgbmImage & xvSgbmImage);
void getRosPCFromSgbmFirmwarePC(
                      sensor_msgs::PointCloud2 &pc2,
                      SgbmImage const &sgbmImage,
                      /*const*/ Slam& xvSlam,
                      pointCloudUnit pUnit,
                      const std::string& frame_id,
                      bool p3dsInWorldFrame);
void getLessRosPCFromSgbmFirmwarePC(
                      sensor_msgs::PointCloud2 &pc2,
                      SgbmImage const &sgbmImage,
                      /*const*/ Slam& xvSlam,
                      pointCloudUnit pUnit,
                      const std::string& frame_id,
                      bool p3dsInWorldFrame);
void getRosPCFromSdkPC( sensor_msgs::PointCloud2 &pc2,
                        std::shared_ptr<PointCloud> xvPointCloud,
                        /*const*/ Slam& xvSlam,
                        pointCloudUnit pUnit,
                        const std::string& frame_id,
                        bool p3dsInWorldFrame);

void RosDevice::initPublisherAdvertise(void)
{
    m_publisher_sgbmCamera_pointCloud = m_nodeHandle_sgbmCamera.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
    
    #ifdef ENABLE_POSE_WITH_CONFIDENCE
    m_publisher_slam_pose = m_nodeHandle_slam.advertise<xv_sdk::PoseStampedConfidence>("pose", 1);
    #else
    m_publisher_slam_pose = m_nodeHandle_slam.advertise<geometry_msgs::PoseStamped>("pose", 1);
    #endif

    m_publisher_slam_path = m_nodeHandle_slam.advertise<nav_msgs::Path>("trajectory", 1,true);
    m_publisher_slam_visualPose = m_nodeHandle_slam.advertise<geometry_msgs::PoseStamped>("visual_pose", 1);
}
        
void RosDevice::init()
{
  /**
   * Device global config
   */

  /// see http://wiki.ros.org/geometry/CoordinateFrameConventions
  std::cout<<"rosDevice::init(),0415/15:01"<<std::endl;
  m_param_tfPrefix = m_rosNamespace;
  m_ddynReconfig->registerVariable<std::string>(
        "tf_prefix",
        m_param_tfPrefix /* callback not called */,
        [this](const std::string& x){GUARD_PARAMS m_param_tfPrefix = x;},
        std::string("prefix for the frames of this device (needed to support multi-device)"));

  m_ddynReconfig->publishServicesTopics();

  /// Static frames (those related to extrinsic calibrations are specified later)

  const Matrix3d rot_optical_to_map = { 0, 0, 1,
                                       -1, 0, 0,
                                        0,-1, 0};
  const Matrix3d rot_map_to_optical = { 0,-1, 0,
                                        0, 0,-1,
                                        1, 0, 0};
  const Pose pose_optical_in_map({0,0,0}, rot_optical_to_map, steady_clock_now());
  const Pose pose_map_in_optical({0,0,0}, rot_map_to_optical, steady_clock_now());
  const Pose pose_identity({0,0,0}, {1,0,0, 0,1,0, 0,0,1}, steady_clock_now());

  /// Put current device's slam-map "map_optical_frame" in the standard ROS frame "map" (for rviz, etc.)

  s_tfStaticBroadcaster->sendTransform(toRosTransformStamped(pose_identity,
                                                             "map",
                                                             m_param_tfPrefix+"/map"));

  s_tfStaticBroadcaster->sendTransform(toRosTransformStamped(pose_optical_in_map,
                                                             m_param_tfPrefix+"/map",
                                                             getFrameId("map_optical_frame")));

  s_tfStaticBroadcaster->sendTransform(toRosTransformStamped(pose_map_in_optical,
                                                             getFrameId("imu_optical_frame"),
                                                             getFrameId("imu_link")));

  /// imu_link same as base_link
  s_tfStaticBroadcaster->sendTransform(toRosTransformStamped(pose_map_in_optical,
                                                             getFrameId("imu_optical_frame"),
                                                             getFrameId("base_link")));

  /**
   * SLAM
   */

  if (m_xvDevice->slam())
  {
    m_server_slam_start = m_nodeHandle_slam.advertiseService("start", &RosDevice::cbSlam_start, this);
    m_server_slam_stop = m_nodeHandle_slam.advertiseService("stop", &RosDevice::cbSlam_stop, this);
    m_server_slam_getPose = m_nodeHandle_slam.advertiseService("get_pose", &RosDevice::cbSlam_getPose, this);
    m_server_slam_getPoseAt = m_nodeHandle_slam.advertiseService("get_pose_at", &RosDevice::cbSlam_getPoseAt, this);

    /// TODO Cslam related services
    /// ...
    ros::Time current_time,last_time;
    current_time = ros::Time::now();
    path_msgs.header.stamp = current_time;//.fromSec(xvPose.hostTimestamp() > 0.1 ? xvPose.hostTimestamp() : 0.1);
    path_msgs.header.frame_id = getFrameId("odom");
    m_xvDevice->slam()->registerCallback([this](const Pose& pose)
    {
      if (pose.hostTimestamp() < 0)
      {
        std::cerr << "XVSDK-ROS-WRAPPER Warning: negative Pose host-timestamp" << std::endl;
        return;
      }
    
    #ifdef ENABLE_SLAM_POSE_FACTOR_CONVERT
      Pose updatePose = m_xvDevice->slam()->poseScaleCalibration(pose);
    #else
      Pose updatePose = pose;
    #endif

    #ifdef USE_SLAM_PATH
      if(m_publisher_slam_path)
      {
          m_publisher_slam_path.publish(toRosPoseStampedRetNavmsgs(updatePose, getFrameId("odom"),path_msgs));
      }
      ros::spinOnce();
      s_tfBroadcaster->sendTransform(toRosTransformStamped(updatePose,
                                                           getFrameId("base_link"),
                                                           getFrameId("odom")));
    #endif
    #ifdef USE_SLAM_POSE
    if(m_publisher_slam_pose)
    {
        #ifdef ENABLE_POSE_WITH_CONFIDENCE
        m_publisher_slam_pose.publish(toRosPoseStampedConfidence(updatePose, getFrameId("map_optical_frame")));
        #else 
        m_publisher_slam_pose.publish(toRosPoseStamped(updatePose, getFrameId("map_optical_frame")));
        #endif
    }

      s_tfBroadcaster->sendTransform(toRosTransformStamped(updatePose,
                                                           getFrameId("map_optical_frame"),
                                                           getFrameId("imu_optical_frame")));
    #endif
    #ifdef ENABLE_INFO_PRINT
      static int k = 0;
      if(k++%500==0){
        k = 1;
        auto pitchYawRoll_u = xv::rotationToPitchYawRoll(updatePose.rotation());
        std::cout << "slam-pose" << timeShowStr(updatePose.edgeTimestampUs(), updatePose.hostTimestamp()) << " (" << updatePose.x() << "," << updatePose.y() << "," << updatePose.z() << "," << pitchYawRoll_u[0]*180/M_PI << "," << pitchYawRoll_u[1]*180/M_PI << "," << pitchYawRoll_u[2]*180/M_PI << ")" << updatePose.confidence() << std::endl;
      }
    #endif
    });
    #if defined(XVSDK_HAS_SLAM_VISUAL_POSE_CALLBACK)
    m_xvDevice->slam()->registerVisualPoseCallback([this](const Pose& pose)
    {
      if (pose.hostTimestamp() < 0)
      {
        std::cerr << "XVSDK-ROS-WRAPPER Warning: negative Pose host-timestamp" << std::endl;
        return;
      }
      if(m_publisher_slam_visualPose)
      {
          m_publisher_slam_visualPose.publish(toRosPoseStamped(pose, getFrameId("map_optical_frame")));
      }

      s_tfBroadcaster->sendTransform(toRosTransformStamped(pose,
                                                           getFrameId("map_optical_frame"),
                                                           getFrameId("imu_optical_frame_unfiltered")));
    });
    #endif

    //m_publisher_slam_lost = m_nodeHandle_slam.advertise<xv_sdk::Lost>("lost", 1);
    //m_xvDevice->slam()->registerLostCallback([this]()
    //{
    //  xv_sdk::Lost lost;
    //  lost.time.now(); /// TODO wait for API to provide time
    //  m_publisher_slam_lost.publish(lost);
    //});
    #ifndef NOT_USE_FE
    m_publisher_slam_stereoPlanes = m_nodeHandle_slam.advertise<xv_sdk::Planes>("stereo_planes", 1);
    m_publisher_slam_stereoPlanesMarkers = m_nodeHandle_slam.advertise<visualization_msgs::MarkerArray>("stereo_planes_markers", 1);
    m_xvDevice->slam()->registerStereoPlanesCallback([this](std::shared_ptr<const std::vector<Plane>> xvPlanes)
    {
      m_publisher_slam_stereoPlanes.publish(toRosPlanes(xvPlanes, getFrameId("map_optical_frame")));
      m_publisher_slam_stereoPlanesMarkers.publish(stereoPlanesToRosMarkerArray(xvPlanes, getFrameId("map_optical_frame")));
    });
    m_publisher_slam_tofPlanes = m_nodeHandle_slam.advertise<xv_sdk::Planes>("tof_planes", 1);
    m_publisher_slam_tofPlanesMarkers = m_nodeHandle_slam.advertise<visualization_msgs::MarkerArray>("tof_planes_markers", 1);
    m_xvDevice->slam()->registerTofPlanesCallback([this](std::shared_ptr<const std::vector<Plane>> xvPlanes)
    {
      m_publisher_slam_tofPlanes.publish(toRosPlanes(xvPlanes, getFrameId("map_optical_frame")));
      m_publisher_slam_tofPlanesMarkers.publish(tofPlanesToRosMarkerArray(xvPlanes, getFrameId("map_optical_frame")));
    });
    m_publisher_slam_mapPoints = m_nodeHandle_slam.advertise<sensor_msgs::PointCloud2>("map_points", 1);
    m_publisher_slam_keyFrames = m_nodeHandle_slam.advertise<visualization_msgs::MarkerArray>("key_frames", 1);
    m_xvDevice->slam()->registerMapCallback([this](std::shared_ptr<const xv::SlamMap> xvSlamMap)
    {
      sensor_msgs::PointCloud2 mapPoints;
      visualization_msgs::MarkerArray keyFrames;
      toRos(*xvSlamMap, mapPoints, keyFrames, getFrameId("map_optical_frame"));
      m_publisher_slam_mapPoints.publish(mapPoints);
      m_publisher_slam_keyFrames.publish(keyFrames);
    });
    #endif
    m_ddynReconfig_slam->registerVariable<bool>(
          "auto_start",
          &m_param_slam_autoStart,
          [](bool){},
          "enable/disable slam auto start when device plugged (true by default)");

    m_ddynReconfig_slam->publishServicesTopics();
    
#ifdef USE_MAPPING_ON_HOST
    std::shared_ptr<xv::Slam> s_slam = nullptr;
    xv::DevicePrivate::s_slamEnableEdgeLocHostMapping = true;
    s_slam = m_xvDevice->slam();

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    if (m_param_slam_autoStart)
      s_slam->start(xv::Slam::Mode::EdgeFusionOnHost);

  
#else
    std::static_pointer_cast<xv::SlamEx>(m_xvDevice->slam())->setEnableOnlineLoopClosure(true);
    if (m_param_slam_autoStart)
      m_xvDevice->slam()->start();
#endif  
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }

  /**
   * IMU
   */

  if (m_xvDevice->imuSensor())
  {
    /// see https://www.ros.org/reps/rep-0145.html
    m_publisher_imuSensor_dataRaw = m_nodeHandle_imuSensor.advertise<sensor_msgs::Imu>("data_raw", 500);
    m_xvDevice->imuSensor()->registerCallback([this](const Imu & xvImu)
    {
      if (xvImu.hostTimestamp < 0)
      {
        std::cerr << "XVSDK-ROS-WRAPPER Warning: negative IMU host-timestamp" << std::endl;
        return;
      }
    
      sensor_msgs::Imu rosImu;
      rosImu.header.stamp.fromSec(xvImu.hostTimestamp > 0.1 ? xvImu.hostTimestamp : 0.1);
      rosImu.header.frame_id = getFrameId("imu_optical_frame");
      rosImu.orientation_covariance = {-1, -1, -1, -1, -1, -1, -1, -1, -1}; /// no orientation data, row major
      rosImu.angular_velocity.x = xvImu.gyro[0];
      rosImu.angular_velocity.y = xvImu.gyro[1];
      rosImu.angular_velocity.z = xvImu.gyro[2];
      const double avcov = m_param_imuSensor_angular_velocity_stddev*m_param_imuSensor_angular_velocity_stddev; /// mutex ?
      rosImu.angular_velocity_covariance = {avcov,0,0,  0,avcov,0,  0,0,avcov}; /// row major
      rosImu.linear_acceleration.x = xvImu.accel[0];
      rosImu.linear_acceleration.y = xvImu.accel[1];
      rosImu.linear_acceleration.z = xvImu.accel[2];
      const double lacov = m_param_imuSensor_linear_acceleration_stddev*m_param_imuSensor_linear_acceleration_stddev; /// mutex ?
      rosImu.linear_acceleration_covariance = {lacov,0,0,  0,lacov,0,  0,0,lacov}; /// row major
      m_publisher_imuSensor_dataRaw.publish(rosImu);
    });

    m_ddynReconfig_imuSensor->registerVariable<double>(
          "linear_acceleration_stddev",
          0. /* callback not called */,
          [this](double x){GUARD_PARAMS m_param_imuSensor_linear_acceleration_stddev = x;},
          "Square root of the linear_acceleration_covariance diagonal elements in m/s^2. Overrides any values reported by the sensor.",
          0., 1e10);

    m_ddynReconfig_imuSensor->registerVariable<double>(
          "angular_velocity_stddev",
          0. /* callback not called */,
          [this](double x){GUARD_PARAMS m_param_imuSensor_angular_velocity_stddev = x;},
          "Square root of the angular_velocity_covariance diagonal elements in rad/s. Overrides any values reported by the sensor.",
          0., 1e10);

    //m_ddynReconfig_imuSensor->registerVariable<double>(
    //      "magnetic_field_stddev",
    //      0. /* callback not called */,
    //      [this](double x){GUARD_PARAMS m_param_imuSensor_magnetic_field_stddev = x;},
    //      "Square root of the magnetic_field_covariance diagonal elements in Tesla. Overrides any values reported by the sensor.",
    //      0., 1e10);
  }

  if (m_xvDevice->orientationStream())
  {
    m_server_imuSensor_startOri = m_nodeHandle_imuSensor.advertiseService("start_orientation", &RosDevice::cbImuSensor_startOri, this);
    m_server_imuSensor_stopOri = m_nodeHandle_imuSensor.advertiseService("stop_orientation", &RosDevice::cbImuSensor_stopOri, this);
    m_server_imuSensor_getOri = m_nodeHandle_imuSensor.advertiseService("get_orientation", &RosDevice::cbImuSensor_getOri, this);
    m_server_imuSensor_getOriAt = m_nodeHandle_imuSensor.advertiseService("get_orientation_at", &RosDevice::cbImuSensor_getOriAt, this);

    /// FIXME
    /// Orientation should be published on topic imu_sensor/data using sensor_msgs::Imu in map_optical_frame (this is an issue).
    /// The C++ API is not designed like this, but getOrientationAt() could be used inside the imu callback above (slow?).
    m_publisher_imuSensor_orientation = m_nodeHandle_imuSensor.advertise<xv_sdk::OrientationStamped>("orientation", 1);
    m_xvDevice->orientationStream()->registerCallback([this](const Orientation & xvOrientation)
    {
      if (xvOrientation.hostTimestamp < 0)
      {
        std::cerr << "XVSDK-ROS-WRAPPER Warning: negative Orientation host-timestamp" << std::endl;
        return;
      }
      
      m_publisher_imuSensor_orientation.publish(toRosOrientationStamped(xvOrientation, getFrameId("map_optical_frame")));
    });

    m_ddynReconfig_imuSensor->registerVariable<double>(
          "orientation_stddev",
          0. /* callback not called */,
          [this](double x){GUARD_PARAMS m_param_imuSensor_orientation_stddev = x;},
          "Square root of the orientation_covariance diagonal elements in rad. Overrides any values reported by the sensor.",
          0., 1e10);
  }

  m_ddynReconfig_imuSensor->publishServicesTopics();

  /**
   * Fisheyes
   */
#ifndef NOT_USE_FE
  if (m_xvDevice->fisheyeCameras())
  {
    //m_publisher_fisheyeCameras_images = m_nodeHandle.advertise<xv_sdk::FisheyeImages>("fisheye_cameras/images", 10);
    //m_publisher_fisheyeCameras_left = m_nodeHandle.advertise<sensor_msgs::Image>("fisheye_cameras/left/image", 10);
    //m_publisher_fisheyeCameras_right = m_nodeHandle.advertise<sensor_msgs::Image>("fisheye_cameras/right/image", 10);
    //m_xvDevice->fisheyeCameras()->registerCallback([this](std::shared_ptr<const FisheyeImages> xvFisheyeImages)
    //{
    //  xv_sdk::FisheyeImages fis;
    //  for (int i = 0; i < int(xvFisheyeImages->images.size()); ++i)
    //  {
    //    const auto& xvGrayImage = xvFisheyeImages->images[i];
    //
    //    fis.images.emplace_back(toRosImage(xvGrayImage, xvFisheyeImages->hostTimestamp));
    //
    //    if (i == 0)
    //      m_publisher_fisheyeCameras_left.publish(fis.images.back());
    //    if (i == 1)
    //      m_publisher_fisheyeCameras_right.publish(fis.images.back());
    //  }
    //  m_publisher_fisheyeCameras_images.publish(fis);
    //});

    m_camInfoMan_fisheye_left = std::make_shared<CamInfoM>(m_nodeHandle_fisheyeLeft, m_rosNamespace+"_fisheye_cameras_left");
    m_imageTransport_fisheye_left = std::make_shared<ImgTpt>(m_nodeHandle_fisheyeLeft);
    m_publisher_fisheyeCameras_leftImage = m_imageTransport_fisheye_left->advertiseCamera("image", 10);

    m_camInfoMan_fisheye_right = std::make_shared<CamInfoM>(m_nodeHandle_fisheyeRight, m_rosNamespace+"_fisheye_cameras_right");
    m_imageTransport_fisheye_right = std::make_shared<ImgTpt>(m_nodeHandle_fisheyeRight);
    m_publisher_fisheyeCameras_rightImage = m_imageTransport_fisheye_right->advertiseCamera("image", 10);

    m_xvFisheyesCalibs = m_xvDevice->fisheyeCameras()->calibration();
    m_fisheyeCameraInfos.resize(m_xvFisheyesCalibs.size());
    for (int i = 0; i < int(m_xvFisheyesCalibs.size()); ++i)
    {
      const auto& calibs = m_xvFisheyesCalibs[i];

      for (const auto& calib : calibs.pdcm)
        m_fisheyeCameraInfos[i][calib.h] = toRosCameraInfo(nullptr, &calib);
      for (const auto& calib : calibs.ucm)
        if (m_fisheyeCameraInfos[i].find(calib.h) == m_fisheyeCameraInfos[i].end())
          m_fisheyeCameraInfos[i][calib.h] = toRosCameraInfo(&calib, nullptr);

      const sensor_msgs::CameraInfo& camInfo = m_fisheyeCameraInfos[i][400];

      const Pose ext(calibs.pose.translation(), calibs.pose.rotation(), steady_clock_now());

      if (i == 0)
      {
        m_camInfoMan_fisheye_left->setCameraInfo(camInfo);
        s_tfStaticBroadcaster->sendTransform(toRosTransformStamped(ext,
                                                                   getFrameId("imu_optical_frame"),
                                                                   getFrameId("fisheye_left_optical_frame")));
      }

      if (i == 1)
      {
        m_camInfoMan_fisheye_right->setCameraInfo(camInfo);
        s_tfStaticBroadcaster->sendTransform(toRosTransformStamped(ext,
                                                                   getFrameId("imu_optical_frame"),
                                                                   getFrameId("fisheye_right_optical_frame")));
      }
    }

    m_xvDevice->fisheyeCameras()->registerCallback([this](const FisheyeImages & xvFisheyeImages)
    {
      for (int i = 0; i < int(xvFisheyeImages.images.size()); ++i)
      {
        const auto& xvGrayImage = xvFisheyeImages.images[i];
        
        if (!xvGrayImage.data)
        {
          std::cerr << "XVSDK-ROS-WRAPPER Warning: no FisheyeImages data" << std::endl;
          return;
        }
        if (xvFisheyeImages.hostTimestamp < 0)
        {
          std::cerr << "XVSDK-ROS-WRAPPER Warning: negative FisheyeImages host-timestamp" << std::endl;
          return;
        }
        
        auto img = toRosImage(xvGrayImage, xvFisheyeImages.hostTimestamp, "");

        if (i == 0)
        {
          img.header.frame_id = getFrameId("fisheye_left_optical_frame");

          sensor_msgs::CameraInfo camInfo = m_camInfoMan_fisheye_left->getCameraInfo();
          if (camInfo.height != img.height)
          {
            m_camInfoMan_fisheye_left->setCameraInfo(m_fisheyeCameraInfos[i][img.height]);
            camInfo = m_fisheyeCameraInfos[i][img.height];
          }
          camInfo.header.frame_id = img.header.frame_id;
          camInfo.header.stamp = img.header.stamp;
          m_publisher_fisheyeCameras_leftImage.publish(img, camInfo);
        }

        if (i == 1)
        {
          img.header.frame_id = getFrameId("fisheye_right_optical_frame");

          sensor_msgs::CameraInfo camInfo = m_camInfoMan_fisheye_right->getCameraInfo();
          if (camInfo.height != img.height)
          {
            m_camInfoMan_fisheye_right->setCameraInfo(m_fisheyeCameraInfos[i][img.height]);
            camInfo = m_fisheyeCameraInfos[i][img.height];
          }
          camInfo.header.frame_id = img.header.frame_id;
          camInfo.header.stamp = img.header.stamp;
          m_publisher_fisheyeCameras_rightImage.publish(img, camInfo);
        }
      }
    });

    m_ddynReconfig_fisheye->registerVariable<bool>(
          "auto_exposure",
          &m_param_fisheye_autoExposure,
          [this](bool x){m_xvDevice->fisheyeCameras()->setExposure(x?0:1);},
          "enable/disable fisheye sensor embedded auto-exposure algorithm (overwrite exposure and gain)");

    m_ddynReconfig_fisheye->registerVariable<int>(
          "exposure",
          2560 /* callback not called */,
          [this](int x){m_xvDevice->fisheyeCameras()->setExposure(1, 0, x*1e-6); m_param_fisheye_autoExposure = false;},
          "[0;9984] fisheye sensors exposure time in µs (auto-exposure disabled if set)",
          0, 9984);

    m_ddynReconfig_fisheye->registerVariable<int>(
          "gain",
          11 /* callback not called */,
          [this](int x){m_xvDevice->fisheyeCameras()->setExposure(1, x); m_param_fisheye_autoExposure = false;},
          "[0;24] fisheye sensors gain (auto-exposure disabled if set)",
          0, 24);

    m_ddynReconfig_fisheye->publishServicesTopics();
  }
#endif

#ifndef NOT_USE_RGB
  /**
   * RGB
   */

  if (m_xvDevice->colorCamera())
  {
    m_server_colorCamera_start = m_nodeHandle_colorCamera.advertiseService("start", &RosDevice::cbColorCamera_start, this);
    m_server_colorCamera_stop = m_nodeHandle_colorCamera.advertiseService("stop", &RosDevice::cbColorCamera_stop, this);

    m_camInfoMan_colorCamera = std::make_shared<CamInfoM>(m_nodeHandle_colorCamera, m_rosNamespace+"_color_camera");
    m_imageTransport_colorCamera = std::make_shared<ImgTpt>(m_nodeHandle_colorCamera);
    m_publisher_colorCamera_imageColor = m_imageTransport_colorCamera->advertiseCamera("image_color", 10);

    if (!m_xvDevice->colorCamera()->calibration().empty())
        m_xvColorCalib = m_xvDevice->colorCamera()->calibration()[0]; /// only one RGB sensor

    for (const auto& calib : m_xvColorCalib.pdcm)
      m_colorCameraInfos[calib.h] = toRosCameraInfo(nullptr, &calib);
    for (const auto& calib : m_xvColorCalib.ucm)
      if (m_colorCameraInfos.find(calib.h) == m_colorCameraInfos.end())
        m_colorCameraInfos[calib.h] = toRosCameraInfo(&calib, nullptr);

    m_camInfoMan_colorCamera->setCameraInfo(m_colorCameraInfos[1080]);

    const Pose ext(m_xvColorCalib.pose.translation(), m_xvColorCalib.pose.rotation(), steady_clock_now());
    s_tfStaticBroadcaster->sendTransform(toRosTransformStamped(ext,
                                                               getFrameId("imu_optical_frame"),
                                                               getFrameId("color_optical_frame")));

    m_xvDevice->colorCamera()->registerCallback([this](const ColorImage & xvColorImage)
    {
      m_lastColorMtx.lock();
      m_last_xvColorImage = xvColorImage;
      m_lastColorMtx.unlock();
      
      if (!xvColorImage.data)
      {
        std::cerr << "XVSDK-ROS-WRAPPER Warning: no ColorImage data" << std::endl;
        return;
      }
      if (xvColorImage.hostTimestamp < 0)
      {
        std::cerr << "XVSDK-ROS-WRAPPER Warning: negative ColorImage host-timestamp" << std::endl;
        return;
      }

      auto img = toRosImage(xvColorImage, getFrameId("color_optical_frame"));

      sensor_msgs::CameraInfo camInfo = m_camInfoMan_colorCamera->getCameraInfo();
      if (camInfo.height != img.height)
      {
        m_camInfoMan_colorCamera->setCameraInfo(m_colorCameraInfos[img.height]);
        camInfo = m_colorCameraInfos[img.height];
      }

      camInfo.header.frame_id = img.header.frame_id;
      camInfo.header.stamp = img.header.stamp;

      m_publisher_colorCamera_imageColor.publish(img, camInfo);
    });

    std::map<std::string, int> resolution_map = {
      {"1920x1080", 0},
      {"1280x720",  1},
      {"640x480",   2},
      {"320x240",   3},
      {"2560x1920", 4},
    };
    m_ddynReconfig_colorCamera->registerEnumVariable<int>(
          "resolution",
          0 /* callback not called */,
          [this](int x){m_xvDevice->colorCamera()->setResolution(xv::ColorCamera::Resolution(x));},
          "color camera resolution",
          resolution_map);

    m_ddynReconfig_colorCamera->registerVariable<int>(
          "framerate",
          30 /* callback not called */,
          [this](int x){m_xvDevice->colorCamera()->setFramerate(x);},
          "color camera frames per second",
          1, 30);

    m_ddynReconfig_colorCamera->publishServicesTopics();

    m_xvDevice->colorCamera()->start();
  }
#endif
  /**
   * SGBM
   */
#ifndef NOT_USE_SGBM
  if (m_xvDevice->sgbmCamera())
  {
    m_server_sgbmCamera_start = m_nodeHandle_sgbmCamera.advertiseService("start", &RosDevice::cbSgbmCamera_start, this);
    m_server_sgbmCamera_stop = m_nodeHandle_sgbmCamera.advertiseService("stop", &RosDevice::cbSgbmCamera_stop, this);

    m_camInfoMan_sgbmCamera = std::make_shared<CamInfoM>(m_nodeHandle_sgbmCamera, m_rosNamespace+"_sgbm_camera");
    m_imageTransport_sgbmCamera = std::make_shared<ImgTpt>(m_nodeHandle_sgbmCamera);
    m_publisher_sgbmCamera_image = m_imageTransport_sgbmCamera->advertiseCamera("image_sgbm", 10);

    if (!m_xvDevice->fisheyeCameras()->calibration().empty())
    {
        m_xvSgbmCalib = m_xvDevice->fisheyeCameras()->calibration()[0]; //temp
  }

 
    sensor_msgs::CameraInfo camInfo = toRosCameraInfo(m_xvSgbmCalib.ucm.empty()?nullptr:&m_xvSgbmCalib.ucm[0],
                                                      m_xvSgbmCalib.pdcm.empty()?nullptr:&m_xvSgbmCalib.pdcm[0]);
    m_camInfoMan_sgbmCamera->setCameraInfo(camInfo);


    const Pose ext(m_xvSgbmCalib.pose.translation(), m_xvSgbmCalib.pose.rotation(), steady_clock_now());
    s_tfStaticBroadcaster->sendTransform(toRosTransformStamped(ext,
                                                               getFrameId("imu_optical_frame"),
                                                               getFrameId("sgbm_optical_frame")));
    #if defined USE_SGBM_IMAGE || defined USE_SGBM_POINTCLOUD                                                            
    m_xvDevice->sgbmCamera()->registerCallback([this](const SgbmImage & xvSgbmImage)
    {
      if(xvSgbmImage.type == xvSgbmImage.Type::Depth)
      {
        #ifdef USE_SGBM_IMAGE
          showSgbmDepthImage(this, xvSgbmImage);
        #endif
        #if (defined USE_SGBM_POINTCLOUD) && (!defined SGBM_FIRMWARE_CLOUD)
          showSgbmPointCloudImage(this, xvSgbmImage);
        #endif
      }
      else if(xvSgbmImage.type == xvSgbmImage.Type::PointCloud)
      {
          #if (defined USE_SGBM_POINTCLOUD) && (defined SGBM_FIRMWARE_CLOUD)
          showSgbmPointCloudImage(this, xvSgbmImage);
          #endif
      }
    });
    #endif
 

    m_ddynReconfig_sgbmCamera->publishServicesTopics();
    struct xv::sgbm_config global_config = {
    1, //enable_dewarp
    1.0, //dewarp_zoom_factor
    0, //enable_disparity
    1, //enable_depth
    #if defined USE_SGBM_POINTCLOUD && defined SGBM_FIRMWARE_CLOUD
    1, //enable_point_cloud
    #else
    0, //enable_point_cloud
    #endif
    0.08, //baseline
    96, //fov
    255, //disparity_confidence_threshold
    {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, //homography
    1, //enable_gamma
    2.2, //gamma_value
    0, //enable_gaussian
    0, //mode
    8000, //max_distance
    100, //min_distance
    };
    m_xvDevice->sgbmCamera()->start(global_config);
   
  }

  //sgbm point cloud
#ifdef USE_SGBM_POINTCLOUD
#ifndef SGBM_PC_WITH_RGB
  if (m_xvDevice->sgbmCamera() && m_xvDevice->slam())
#else
  if (m_xvDevice->sgbmCamera() && m_xvDevice->colorCamera() && m_xvDevice->slam())
#endif
  {
    /**
     * Color for Sgbm
     */
    #ifdef SGBM_PC_WITH_RGB
    m_publisher_sgbmCamera_imageColor = m_imageTransport_sgbmCamera->advertiseCamera("image_color", 10);
    #endif
  }
#endif
#endif

#ifndef NOT_USE_TOF
  /**
   * ToF
   */

  if (m_xvDevice->tofCamera())
  {
    m_server_tofCamera_start = m_nodeHandle_tofCamera.advertiseService("start", &RosDevice::cbTofCamera_start, this);
    m_server_tofCamera_stop = m_nodeHandle_tofCamera.advertiseService("stop", &RosDevice::cbTofCamera_stop, this);

    m_camInfoMan_tofCamera = std::make_shared<CamInfoM>(m_nodeHandle_tofCamera, m_rosNamespace+"_tof_camera");
    m_imageTransport_tofCamera = std::make_shared<ImgTpt>(m_nodeHandle_tofCamera);
    m_publisher_tofCamera_image = m_imageTransport_tofCamera->advertiseCamera("image", 10); /// image_raw if integer [mm]

    /// Only one ToF sensor and one resolution
    if (!m_xvDevice->tofCamera()->calibration().empty())
        m_xvTofCalib = m_xvDevice->tofCamera()->calibration()[0];
    sensor_msgs::CameraInfo camInfo = toRosCameraInfo(m_xvTofCalib.ucm.empty()?nullptr:&m_xvTofCalib.ucm[0],
                                                      m_xvTofCalib.pdcm.empty()?nullptr:&m_xvTofCalib.pdcm[0]);
    m_camInfoMan_tofCamera->setCameraInfo(camInfo);

    const Pose ext(m_xvTofCalib.pose.translation(), m_xvTofCalib.pose.rotation(), steady_clock_now());
    s_tfStaticBroadcaster->sendTransform(toRosTransformStamped(ext,
                                                               getFrameId("imu_optical_frame"),
                                                               getFrameId("tof_optical_frame")));
                                                              
    // bool ret = m_xvDevice->tofCamera()->setLibWorkMode(xv::TofCamera::SonyTofLibMode::LABELIZE_SF);
    // if(!ret)
    // {
    //   std::cout<<"set TOF LibWorkMode failed"<<std::endl;
    //}

    //default is Edge mode 
    #ifdef TOF_QVGA
    bool ret = m_xvDevice->tofCamera()->setSonyTofSetting(xv::TofCamera::SonyTofLibMode::LABELIZE_SF,
                                        xv::TofCamera::Resolution::QVGA,
                                        xv::TofCamera::Framerate::FPS_30);
    #else//VGA
    bool ret = m_xvDevice->tofCamera()->setSonyTofSetting(xv::TofCamera::SonyTofLibMode::LABELIZE_SF,
                                        xv::TofCamera::Resolution::VGA,
                                        xv::TofCamera::Framerate::FPS_30);
    #endif
    if(!ret)
    {
        std::cout<<"set TOF Setting failed"<<std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    m_xvDevice->tofCamera()->registerCallback([this](const DepthImage & xvDepthImage)
    {
      if (!xvDepthImage.data)
      {
        std::cerr << "XVSDK-ROS-WRAPPER Warning: no DepthImage data" << std::endl;
        return;
      }
      if (xvDepthImage.hostTimestamp < 0)
      {
        std::cerr << "XVSDK-ROS-WRAPPER Warning: negative DepthImage host-timestamp" << std::endl;
        return;
      }

      // ignore IR
      if (xvDepthImage.type != xv::DepthImage::Type::Depth_16 && xvDepthImage.type != xv::DepthImage::Type::Depth_32) {
        return;
      }

      auto img = toRosImage(xvDepthImage, getFrameId("tof_optical_frame"));

      sensor_msgs::CameraInfo camInfo = m_camInfoMan_tofCamera->getCameraInfo();
      camInfo.header.frame_id = img.header.frame_id;
      camInfo.header.stamp = img.header.stamp;

      m_publisher_tofCamera_image.publish(img, camInfo);
    });
    
    m_ddynReconfig_tofCamera->registerVariable<bool>(
          "use_map_frame",
          &m_param_tofCamera_useMapFrame,
          [this](bool){},
          "select ToF point cloud frame: true to express in map (world) frame, false to express in local ToF frame");

    m_ddynReconfig_tofCamera->publishServicesTopics();

    m_xvDevice->tofCamera()->start();
  }

  /**
   * RGBD
   */
  #ifndef NOT_USE_RGB
  #ifndef NOT_USE_TOF
  if (m_xvDevice->tofCamera())
  {
    m_server_rgbd_start = m_nodeHandle.advertiseService("rgbd_camera/start", &RosDevice::cbRgbd_start, this);
    m_server_rgbd_stop = m_nodeHandle.advertiseService("rgbd_camera/stop", &RosDevice::cbRgbd_stop, this);

    ros::NodeHandle rgbdNode(m_nodeHandle, "rgbd_camera");
    m_camInfoMan_rgbd = std::make_shared<CamInfoM>(rgbdNode, m_rosNamespace+"_rgbd");
    m_imageTransport_rgbd = std::make_shared<ImgTpt>(rgbdNode);
    m_publisher_rgbd_image = m_imageTransport_rgbd->advertiseCamera("image", 10); /// image_raw if integer [mm]

    /// Only one ToF sensor and one resolution
    if (!m_xvDevice->tofCamera()->calibration().empty())
        m_xvTofCalib = m_xvDevice->tofCamera()->calibration()[0];
    sensor_msgs::CameraInfo camInfo = toRosCameraInfo(m_xvTofCalib.ucm.empty()?nullptr:&m_xvTofCalib.ucm[0],
                                                      m_xvTofCalib.pdcm.empty()?nullptr:&m_xvTofCalib.pdcm[0]);
    m_camInfoMan_rgbd->setCameraInfo(camInfo);

    m_xvDevice->tofCamera()->registerColorDepthImageCallback([this](const DepthColorImage & xvDepthColorImage)
    {
      auto img = toRosImage(xvDepthColorImage, getFrameId("tof_optical_frame"));

      sensor_msgs::CameraInfo camInfo = m_camInfoMan_rgbd->getCameraInfo();
      camInfo.header.frame_id = img.header.frame_id;
      camInfo.header.stamp = img.header.stamp;

      m_publisher_rgbd_image.publish(img, camInfo);
    });

  }
  #endif
  #endif
  /**
   * Virtual sensors
   */
#ifdef TOF_PC_WITH_RGB
  if (m_xvDevice->tofCamera() && m_xvDevice->colorCamera() && m_xvDevice->slam())
#else
  if (m_xvDevice->tofCamera() && m_xvDevice->slam())
#endif
  {
    /**
     * Color for ToF
     */
    #ifdef TOF_PC_WITH_RGB
    m_publisher_tofCamera_imageColor = m_imageTransport_tofCamera->advertiseCamera("image_color", 10);
    #endif
    m_publisher_tofCamera_pointCloud = m_nodeHandle_tofCamera.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);

    /// XXX Need its thread ?
    m_xvDevice->tofCamera()->registerCallback([this](const DepthImage & xvDepthImage)
    {
      // ignore IR
      if (xvDepthImage.type != xv::DepthImage::Type::Depth_16 && xvDepthImage.type != xv::DepthImage::Type::Depth_32) {
        return;
      }

      #ifdef TOF_PC_WITH_RGB
      m_lastColorMtx.lock();
      auto last_xvColorImage = m_last_xvColorImage;
      m_lastColorMtx.unlock();
     
      if (!last_xvColorImage.data)
      {
        std::cerr << "XVSDK-ROS-WRAPPER Warning: no ColorImage data" << std::endl;
        return;
      }
      if (last_xvColorImage.hostTimestamp < 0)
      {
        std::cerr << "XVSDK-ROS-WRAPPER Warning: negative ColorImage host-timestamp" << std::endl;
        return;
      }
      #endif
      if (!xvDepthImage.data)
      {
        std::cerr << "XVSDK-ROS-WRAPPER Warning: no DepthImage data" << std::endl;
        return;
      }
      if (xvDepthImage.hostTimestamp < 0)
      {
        std::cerr << "XVSDK-ROS-WRAPPER Warning: negative DepthImage host-timestamp" << std::endl;
        return;
      }
      
      #ifdef TOF_PC_WITH_RGB
      const bool useMapFrame = m_param_tofCamera_useMapFrame;

      cv::Mat color_for_tof;
      std::vector<Vector3d> p3ds_in_world;
      computePointCloud(xvDepthImage,
                        last_xvColorImage,
                        *m_xvDevice->slam(),
                        m_xvTofCalib,
                        m_xvColorCalib,
                        color_for_tof,
                        p3ds_in_world,
                        useMapFrame);

      auto rosColor = toRosImage(color_for_tof,
                                 xvDepthImage.hostTimestamp,
                                 getFrameId("tof_optical_frame"));
      sensor_msgs::CameraInfo camInfo = m_camInfoMan_tofCamera->getCameraInfo();
      camInfo.header.frame_id = rosColor.header.frame_id;
      camInfo.header.stamp = rosColor.header.stamp;
      m_publisher_tofCamera_imageColor.publish(rosColor, camInfo);

      auto rosPC2 = toRosPointCloud2(color_for_tof,
                                     p3ds_in_world,
                                     xvDepthImage.hostTimestamp,
                                     useMapFrame ? getFrameId("map_optical_frame") : getFrameId("tof_optical_frame"));

      #else/*not with rgb*/

      auto pointcloud = m_xvDevice->tofCamera()->depthImageToPointCloud(xvDepthImage);
      std::vector<Vector3d> p3ds_in_world;
      const bool useMapFrame = m_param_sgbmCamera_useMapFrame;
      computePointCloud(pointcloud,
                        *m_xvDevice->slam(),
                        p3ds_in_world,
                        (m_xvDevice->tofCamera()->getManufacturer() == xv::TofCamera::Manufacturer::Sony)?
                        (pointCloudUnit::m):(pointCloudUnit::mm),
                        useMapFrame);

      auto rosPC2 = toRosPointCloud2(p3ds_in_world,
                                    xvDepthImage.hostTimestamp,
                                     useMapFrame ? getFrameId("map_optical_frame") : getFrameId("tof_optical_frame"));

      #endif
      m_publisher_tofCamera_pointCloud.publish(rosPC2);
    });
    
  }
  #endif
}

bool RosDevice::cbSlam_start(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
{
  bool ok = m_xvDevice->slam()->start();
  res.success = ok;
  res.message = ok ? "Slam started" : "failed";
  return ok;
}

bool RosDevice::cbSlam_stop(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
{
  bool ok = m_xvDevice->slam()->stop();
  res.success = ok;
  res.message = ok ? "Slam stopped" : "failed";
  return ok;
}

bool RosDevice::cbSlam_getPose(xv_sdk::GetPose::Request& req, xv_sdk::GetPose::Response& res)
{
  Pose pose;
  bool ok = m_xvDevice->slam()->getPose(pose, req.prediction.toSec());
  if (ok)
  {
    res.pose = toRosPoseStamped(pose, getFrameId("map_optical_frame"));
  }

  return ok;
}

bool RosDevice::cbSlam_getPoseAt(xv_sdk::GetPoseAt::Request& req, xv_sdk::GetPoseAt::Response& res)
{
  Pose pose;
  bool ok = m_xvDevice->slam()->getPoseAt(pose, req.timestamp.toSec());
  if (ok)
  {
    res.pose = toRosPoseStamped(pose, getFrameId("map_optical_frame"));
  }

  return ok;
}

bool RosDevice::cbImuSensor_startOri(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
{
  bool ok = m_xvDevice->orientationStream()->start();
  res.success = ok;
  res.message = ok ? "Orientation started" : "failed";
  return ok;
}

bool RosDevice::cbImuSensor_stopOri(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
{
  bool ok = m_xvDevice->orientationStream()->stop();
  res.success = ok;
  res.message = ok ? "Orientation stopped" : "failed";
  return ok;
}

bool RosDevice::cbImuSensor_getOri(xv_sdk::GetOrientation::Request& req, xv_sdk::GetOrientation::Response& res)
{
  Orientation ori;
  bool ok = m_xvDevice->orientationStream()->get(ori, req.prediction.toSec());
  if (ok)
  {
    res.orientation = toRosOrientationStamped(ori, getFrameId("map_optical_frame"));
  }

  return ok;
}

bool RosDevice::cbImuSensor_getOriAt(xv_sdk::GetOrientationAt::Request& req, xv_sdk::GetOrientationAt::Response& res)
{
  Orientation ori;
  bool ok = m_xvDevice->orientationStream()->getAt(ori, req.timestamp.toSec());
  if (ok)
  {
    res.orientation = toRosOrientationStamped(ori, getFrameId("map_optical_frame"));
  }

  return ok;
}

bool RosDevice::cbColorCamera_start(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
{
  bool ok = m_xvDevice->colorCamera()->start();
  res.success = ok;
  res.message = ok ? "Color camera started" : "failed";
  return ok;
}

bool RosDevice::cbColorCamera_stop(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
{
  bool ok = m_xvDevice->colorCamera()->stop();
  res.success = ok;
  res.message = ok ? "Color camera stopped" : "failed";
  return ok;
}

bool RosDevice::cbSgbmCamera_start(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  bool ok = m_xvDevice->sgbmCamera()->start("");
  res.success = ok;
  res.message = ok ? "Sgbm camera started" : "failed";
  return ok;
}

bool RosDevice::cbSgbmCamera_stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  bool ok = m_xvDevice->sgbmCamera()->stop();
  res.success = ok;
  res.message = ok ? "Sgbm camera stopped" : "failed";
  return ok;
}

bool RosDevice::cbTofCamera_start(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
{
  bool ok = m_xvDevice->tofCamera()->start();
  res.success = ok;
  res.message = ok ? "ToF camera started" : "failed";
  return ok;
}

bool RosDevice::cbTofCamera_stop(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
{
  bool ok = m_xvDevice->tofCamera()->stop();
  res.success = ok;
  res.message = ok ? "ToF camera stopped" : "failed";
  return ok;
}

bool RosDevice::cbRgbd_start(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
{
  // XXX temply for sony tof
  //m_xvDevice->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_1280x720);

  bool ok1 = m_xvDevice->tofCamera()->start();
  bool ok2 = m_xvDevice->colorCamera()->start();
  res.success = ok1 && ok2;
  res.message = res.success ? "ToF camera started" : "failed";
  return res.success;
}

bool RosDevice::cbRgbd_stop(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
{
  bool ok1 = m_xvDevice->tofCamera()->stop();
  bool ok2 = m_xvDevice->colorCamera()->stop();
  res.success = ok1 && ok2;
  res.message = res.success ? "ToF camera stopped" : "failed";
  return res.success;
}


void RosWrapper::watchDevices()
{
  while (!m_stopWatchDevices)
  {
    std::map<std::string, std::shared_ptr<Device> > device_map;

    std::string json = "";
    std::string jsonPath = "/etc/xvisio/config.json";
    std::ifstream ifs( jsonPath );
    if( ifs.is_open() ){
        std::stringstream fbuf;
        fbuf << ifs.rdbuf();
        json = fbuf.str();
        ifs.close();
        device_map = getDevices(0., json);
    } else {
      #ifdef ENABLE_INFO_PRINT
        std::cout << jsonPath << " doesn't exists, use default json." << std::endl;
      #endif
        device_map = getDevices(0.);
    }

    //xv::setLogLevel(xv::LogLevel::debug);

    std::vector<std::string> serialNumbers;
    for (auto& pair: device_map)
      serialNumbers.emplace_back(pair.first);

    {
      std::lock_guard<std::mutex> lock(m_deviceMapMutex);
      for (const std::string& serialNumber : serialNumbers)
      {
        auto it = m_deviceMap.find(serialNumber);
        if (it == m_deviceMap.end())
        {
          std::string rosNamespace = "xv_dev";
          if (m_uuidLength > 0)
          {
            /// Replace unsupported characters by "_" and ensure it starts with a letter
            std::regex e ("[^a-zA-Z0-9/_]{1}");
            rosNamespace = "uuid_" + std::regex_replace(serialNumber, e,"_").substr(0, m_uuidLength);
          }

          m_deviceMap.emplace(serialNumber, std::make_shared<RosDevice>(m_topNodeHandle, rosNamespace, device_map[serialNumber]));
          publishNewDevice(rosNamespace);
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

RosWrapper::RosWrapper(ros::NodeHandle *nh)
    : m_topNodeHandle(nh)
    , m_ddynReconfig(new ddynamic_reconfigure::DDynamicReconfigure(*m_topNodeHandle))
{
  s_tfBroadcaster.reset(new tf2_ros::TransformBroadcaster);
  s_tfStaticBroadcaster.reset(new tf2_ros::StaticTransformBroadcaster);

  m_server_getDevices = m_topNodeHandle->advertiseService("get_devices", &RosWrapper::cbGetDevices, this);
  m_publisher_newDevice = m_topNodeHandle->advertise<std_msgs::String>("new_device", 10);

  /**
   * Config
   */

  /// Static
  m_topNodeHandle->getParam("uuid_length", m_uuidLength);

  /// Dynamic: frames (need a mutex ?)
  for (const auto& pair : s_frameIds)
  {
    const std::string default_name = pair.first;
    m_ddynReconfig->registerVariable<std::string>(
          default_name,
          default_name /* callback not called */,
          [&,default_name](const std::string& x){GUARD_PARAMS s_frameIds[default_name] = x;},
          "custom name for frame "+default_name);
  }

  m_ddynReconfig->publishServicesTopics();

  /**
   * Start device detection
   */

  m_stopWatchDevices = false;
  m_watchDevicesThread = std::thread(&RosWrapper::watchDevices, this);
}

RosWrapper::~RosWrapper()
{
  m_stopWatchDevices = true;
  if (m_watchDevicesThread.joinable())
    m_watchDevicesThread.join();
}

bool RosWrapper::cbGetDevices(xv_sdk::GetDevices::Request& /*req*/, xv_sdk::GetDevices::Response& res)
{
  std::lock_guard<std::mutex> lock(m_deviceMapMutex);
  for (const auto& pair : m_deviceMap)
    res.devices.emplace_back(pair.second->getRosNamespace());

  return true;
}

void RosWrapper::publishNewDevice(const std::string &rosNamespace)
{
  std_msgs::String msg;
  msg.data = rosNamespace;
  m_publisher_newDevice.publish(msg);
}
void showSgbmDepthImage(const RosDevice* device, const SgbmImage & xvSgbmImage)
{
    if(xvSgbmImage.type == xvSgbmImage.Type::Depth)
    {
        if (!xvSgbmImage.data)
        {
          std::cerr << "XVSDK-ROS-WRAPPER Warning: no SgbmImage data" << std::endl;
          return;
        }

        if (xvSgbmImage.hostTimestamp < 0)
        {
          std::cerr << "XVSDK-ROS-WRAPPER Warning: negative SgbmImage host-timestamp" << std::endl;
          return;
        }
        #ifdef ENABLE_INFO_PRINT
        static int j = 0;
        if(j++%50==0){
          std::string sgbmtype_str = ((int)xvSgbmImage.type == 0)?("Disparity"):(((int)xvSgbmImage.type == 1)?("Depth"):("PointCloud"));
          std::cout << "xvSgbmImage data:" <<sgbmtype_str<<", width:"<<xvSgbmImage.width<<",height:"<<xvSgbmImage.height<<",timstamp:"<< xvSgbmImage.edgeTimestampUs<<"," << xvSgbmImage.hostTimestamp<< std::endl;
        }
        #endif 
      
        auto img = toRosImage(xvSgbmImage, device->getFrameId("sgbm_optical_frame"));

        sensor_msgs::CameraInfo camInfo = device->m_camInfoMan_sgbmCamera->getCameraInfo();

        camInfo.header.frame_id = img.header.frame_id;
        camInfo.header.stamp = img.header.stamp;

        device->m_publisher_sgbmCamera_image.publish(img, camInfo);
    }
}

void showSgbmPointCloudImage(RosDevice* device, const SgbmImage & xvSgbmImage)
{
    #ifdef DISPLAY_POINT_CLOUD_FPS
    static FpsCount fc;
    fc.tic();
    static int k = 0;
    if(k++%1000==0)
    {
        std::cout << "poind cloud:"
                << std::round(fc.fps()) << "fps"
                << std::endl;
                k = 0;
    }
    #endif
    std::string sgbmtype_str = ((int)xvSgbmImage.type == 0)?("Disparity"):(((int)xvSgbmImage.type == 1)?("Depth"):("PointCloud"));
    // ignore 

    #ifdef SGBM_PC_WITH_RGB
    device->m_lastColorMtx.lock();
    auto last_xvColorImage = device->m_last_xvColorImage;
    device->m_lastColorMtx.unlock();
    
    if (!last_xvColorImage.data)
    {
      std::cerr << "XVSDK-ROS-WRAPPER Warning: no ColorImage data" << std::endl;
      return;
    }
    if (last_xvColorImage.hostTimestamp < 0)
    {
        std::cerr << "XVSDK-ROS-WRAPPER Warning: negative ColorImage host-timestamp" << std::endl;
        return;
    }
    #endif
    if (!xvSgbmImage.data)
    {
      std::cerr << "XVSDK-ROS-WRAPPER Warning: no DepthImage data" << std::endl;
      return;
    }
    if (xvSgbmImage.hostTimestamp < 0)
    {
      std::cerr << "XVSDK-ROS-WRAPPER Warning: negative DepthImage host-timestamp" << std::endl;
      return;
    }

  #ifndef SGBM_PC_WITH_RGB
        #ifdef SGBM_FIRMWARE_CLOUD
        const bool useMapFrame = device->m_param_sgbmCamera_useMapFrame;
        if(xvSgbmImage.type != SgbmImage::Type::PointCloud)
        {
            return;
        }
         
        // std::string frameId = useMapFrame ? device->getFrameId("map_optical_frame") : device->getFrameId("sgbm_optical_frame");
        sensor_msgs::PointCloud2 rosPC2;
        #ifdef ENABLE_LESS_POINT_CLOUD
        getLessRosPCFromSgbmFirmwarePC(
                        rosPC2,
                        xvSgbmImage,
                        *device->m_xvDevice->slam(),
                        pointCloudUnit::m,
                        useMapFrame ? device->getFrameId("map_optical_frame") : device->getFrameId("sgbm_optical_frame"),
                        useMapFrame);
        #else
        getRosPCFromSgbmFirmwarePC(
                        rosPC2,
                        xvSgbmImage,
                        *device->m_xvDevice->slam(),
                        pointCloudUnit::m,
                        useMapFrame ? device->getFrameId("map_optical_frame") : device->getFrameId("sgbm_optical_frame"),
                        useMapFrame);
        #endif

        #else
        if(xvSgbmImage.type != SgbmImage::Type::Depth)
        {
            return;
        }
        auto pointcloud = device->m_xvDevice->sgbmCamera()->depthImageToPointCloud(xvSgbmImage);
        // std::vector<Vector3d> p3ds_in_world;
        const bool useMapFrame = device->m_param_sgbmCamera_useMapFrame;
        // computePointCloud(pointcloud,
        //                   *device->m_xvDevice->slam(),
        //                   p3ds_in_world,
        //                   pointCloudUnit::m,
        //                   useMapFrame);
        // auto rosPC2 = toRosPointCloud2(p3ds_in_world,
        //             xvSgbmImage.hostTimestamp,
        //             useMapFrame ? device->getFrameId("map_optical_frame") : device->getFrameId("sgbm_optical_frame"));
        sensor_msgs::PointCloud2 rosPC2;
        getRosPCFromSdkPC(rosPC2,
                          pointcloud,
                          *device->m_xvDevice->slam(),
                          pointCloudUnit::m,
                          useMapFrame ? device->getFrameId("map_optical_frame") : device->getFrameId("sgbm_optical_frame"),
                          useMapFrame);
        #endif

        // not dewarp use
        // const bool useMapFrame = m_param_sgbmCamera_useMapFrame;
        // std::vector<Vector3d> p3ds_in_world;
        // computePointCloud(xvSgbmImage,
        //                   *m_xvDevice->slam(),
        //                   m_xvSgbmCalib,
        //                   p3ds_in_world,
        //                   useMapFrame);
        // auto rosPC2 = toRosPointCloud2(p3ds_in_world,
        //                               xvSgbmImage.hostTimestamp,
        //                                useMapFrame ? getFrameId("map_optical_frame") : getFrameId("sgbm_optical_frame"));
        
  #else//with RGB
        const bool useMapFrame = device->m_param_sgbmCamera_useMapFrame;
        cv::Mat color_for_sgbm;
        std::vector<Vector3d> p3ds_in_world;
        computePointCloud(xvSgbmImage,
                          last_xvColorImage,
                          *device->m_xvDevice->slam(),
                          device->m_xvSgbmCalib,
                          device->m_xvColorCalib,
                          color_for_sgbm,
                          p3ds_in_world,
                          useMapFrame);
        auto rosColor = toRosImage(color_for_sgbm,
                                  xvSgbmImage.hostTimestamp,
                                  device->getFrameId("sgbm_optical_frame"));
                                  
        sensor_msgs::CameraInfo camInfo = device->m_camInfoMan_sgbmCamera->getCameraInfo();
        camInfo.header.frame_id = rosColor.header.frame_id;
        camInfo.header.stamp = rosColor.header.stamp;
        device->m_publisher_sgbmCamera_imageColor.publish(rosColor, camInfo);
        auto rosPC2 = toRosPointCloud2(color_for_sgbm,
                                      p3ds_in_world,
                                      xvSgbmImage.hostTimestamp,
                                      useMapFrame ? device->getFrameId("map_optical_frame") : device->getFrameId("sgbm_optical_frame"));
  #endif
    if(device->m_publisher_sgbmCamera_pointCloud)
    {
        device->m_publisher_sgbmCamera_pointCloud.publish(rosPC2);
    }
}

void getRosPCFromSdkPC(
                      sensor_msgs::PointCloud2 &pc2,
                      std::shared_ptr<PointCloud> xvPointCloud,
                      /*const*/ Slam& xvSlam,
                      pointCloudUnit pUnit,
                      const std::string& frame_id,
                      bool p3dsInWorldFrame)
{
    #ifdef CONVERT_TO_WORLD_COORDINATE_SYSTEM
    Pose pose_at_pc_time;
    if (!xvSlam.getPoseAt(pose_at_pc_time, xvPointCloud->hostTimestamp))
    {
      pose_at_pc_time.setTranslation({0,0,0});
      pose_at_pc_time.setRotation({1,0,0, 0,1,0, 0,0,1});
    }
    auto sgbm_pose_in_world = pose_at_pc_time ;//* pc_calibration.pose;
    #endif

    auto genFields = []()
    {
      std::vector<sensor_msgs::PointField> fds;
      sensor_msgs::PointField f;

      f.name = "x";
      f.offset = 0;
      f.datatype = sensor_msgs::PointField::FLOAT32;
      f.count = 1;
      fds.emplace_back(f);
      f.name = "y";
      f.offset = 4;
      f.datatype = sensor_msgs::PointField::FLOAT32;
      f.count = 1;
      fds.emplace_back(f);
      f.name = "z";
      f.offset = 8;
      f.datatype = sensor_msgs::PointField::FLOAT32;
      f.count = 1;
      fds.emplace_back(f);

      return fds;
    };
    static const std::vector<sensor_msgs::PointField> fields = genFields();
    struct __attribute__((packed)) F {float x,y,z;};
    pc2.header.stamp.fromSec(steady_clock_now());
    pc2.header.frame_id = frame_id;
    pc2.height = 1;
    const int nbPt = xvPointCloud->points.size();
    pc2.width = nbPt;
    pc2.fields = fields; /// xyz
    pc2.is_bigendian = false;
    pc2.point_step = 3*sizeof(float);
    pc2.row_step = pc2.point_step * pc2.width;
    pc2.is_dense = true;

    pc2.data.resize(nbPt*pc2.point_step);
    F* data = reinterpret_cast<F*>(pc2.data.data());

    unsigned int size = xvPointCloud->points.size();
    Vector3d p3d_in_sgbm;
    #ifdef CONVERT_TO_WORLD_COORDINATE_SYSTEM
    Vector3d p3d_in_world;
    #endif
    for (unsigned int i=0; i < size; i++, data++)
    {
        #ifdef SGBM_POINTCLOUD_UNIT_M
        p3d_in_sgbm = { (double(xvPointCloud->points[i][0]/1000.0)), 
                        (double(xvPointCloud->points[i][1]/1000.0)), 
                        (double(xvPointCloud->points[i][2]/1000.0))};
        #else
        p3d_in_sgbm = { (double(xvPointCloud->points[i][0])), 
                        (double(xvPointCloud->points[i][1])), 
                        (double(xvPointCloud->points[i][2]))};
        #endif
        
        #ifdef CONVERT_TO_WORLD_COORDINATE_SYSTEM
        p3d_in_world = Transform(sgbm_pose_in_world.translation(), sgbm_pose_in_world.rotation()) * p3d_in_sgbm;
        if (p3dsInWorldFrame)
        {
            *data = F { float(p3d_in_world[0]), float(p3d_in_world[1]), float(p3d_in_world[2]) };
        }
        else
        {
            *data = F { float(p3d_in_sgbm[0]), float(p3d_in_sgbm[1]), float(p3d_in_sgbm[2]) };
        }
        #else
        *data = F { float(p3d_in_sgbm[0]), float(p3d_in_sgbm[1]), float(p3d_in_sgbm[2]) };
        #endif
    }
}

void getRosPCFromSgbmFirmwarePC(
                      sensor_msgs::PointCloud2 &pc2,
                      SgbmImage const &sgbmImage,
                      /*const*/ Slam& xvSlam,
                      pointCloudUnit pUnit,
                      const std::string& frame_id,
                      bool p3dsInWorldFrame)               
{
    if(sgbmImage.type != xv::SgbmImage::Type::PointCloud)
    {
        // return;
    }
    #ifdef CONVERT_TO_WORLD_COORDINATE_SYSTEM
    Pose pose_at_pc_time;
    if (!xvSlam.getPoseAt(pose_at_pc_time, sgbmImage.hostTimestamp))
    {
        pose_at_pc_time.setTranslation({0,0,0});
        pose_at_pc_time.setRotation({1,0,0, 0,1,0, 0,0,1});
    }
    auto sgbm_pose_in_world = pose_at_pc_time ;//* pc_calibration.pose;
    #endif
    auto genFields = []()
    {
      std::vector<sensor_msgs::PointField> fds;
      sensor_msgs::PointField f;

      f.name = "x";
      f.offset = 0;
      f.datatype = sensor_msgs::PointField::FLOAT32;
      f.count = 1;
      fds.emplace_back(f);
      f.name = "y";
      f.offset = 4;
      f.datatype = sensor_msgs::PointField::FLOAT32;
      f.count = 1;
      fds.emplace_back(f);
      f.name = "z";
      f.offset = 8;
      f.datatype = sensor_msgs::PointField::FLOAT32;
      f.count = 1;
      fds.emplace_back(f);

      return fds;
    };
    static const std::vector<sensor_msgs::PointField> fields = genFields();
    struct __attribute__((packed)) F {float x,y,z;};
    pc2.header.stamp.fromSec(steady_clock_now());
    pc2.header.frame_id = frame_id;
    pc2.height = 1;
    const int nbPt = sgbmImage.width * sgbmImage.height;
    pc2.width = nbPt;
    pc2.fields = fields; /// xyz
    pc2.is_bigendian = false;
    pc2.point_step = 3*sizeof(float);
    pc2.row_step = pc2.point_step * pc2.width;
    pc2.is_dense = true;

    pc2.data.resize(nbPt*pc2.point_step);
    F* data = reinterpret_cast<F*>(pc2.data.data());

    short *dataTemp = (short *)sgbmImage.data.get();
    unsigned int size = sgbmImage.width * sgbmImage.height;
    Vector3d p3d_in_sgbm;
    for (unsigned int i=0; i < size; i++, data++)
    {
        #ifdef SGBM_POINTCLOUD_UNIT_M
        p3d_in_sgbm = { (double((*dataTemp)/1000.0)), 
                         double((*(dataTemp + 1))/1000.0), 
                         double((*(dataTemp + 2))/1000.0)};
        #else
        p3d_in_sgbm = { double(*dataTemp), 
                        double(*(dataTemp + 1)), 
                        double(*(dataTemp + 2))};
        #endif

        #ifdef CONVERT_TO_WORLD_COORDINATE_SYSTEM
        const Vector3d p3d_in_world = Transform(sgbm_pose_in_world.translation(), sgbm_pose_in_world.rotation()) * p3d_in_sgbm;
        if (p3dsInWorldFrame)
        {
            *data = F { float(p3d_in_world[0]), float(p3d_in_world[1]), float(p3d_in_world[2]) };
        }
        else
        {
            *data = F { float(p3d_in_sgbm[0]), float(p3d_in_sgbm[1]), float(p3d_in_sgbm[2]) };
        }
        #else
        *data = F { float(p3d_in_sgbm[0]), float(p3d_in_sgbm[1]), float(p3d_in_sgbm[2]) };
        #endif
        dataTemp += 3;
    }
}

void getLessRosPCFromSgbmFirmwarePC(
                      sensor_msgs::PointCloud2 &pc2,
                      SgbmImage const &sgbmImage,
                      /*const*/ Slam& xvSlam,
                      pointCloudUnit pUnit,
                      const std::string& frame_id,
                      bool p3dsInWorldFrame)               
{
    if(sgbmImage.type != xv::SgbmImage::Type::PointCloud)
    {
        // return;
    }
    #ifdef CONVERT_TO_WORLD_COORDINATE_SYSTEM
    Pose pose_at_pc_time;
    if (!xvSlam.getPoseAt(pose_at_pc_time, sgbmImage.hostTimestamp))
    {
        pose_at_pc_time.setTranslation({0,0,0});
        pose_at_pc_time.setRotation({1,0,0, 0,1,0, 0,0,1});
    }
    auto sgbm_pose_in_world = pose_at_pc_time ;//* pc_calibration.pose;
    #endif
    auto genFields = []()
    {
      std::vector<sensor_msgs::PointField> fds;
      sensor_msgs::PointField f;

      f.name = "x";
      f.offset = 0;
      f.datatype = sensor_msgs::PointField::FLOAT32;
      f.count = 1;
      fds.emplace_back(f);
      f.name = "y";
      f.offset = 4;
      f.datatype = sensor_msgs::PointField::FLOAT32;
      f.count = 1;
      fds.emplace_back(f);
      f.name = "z";
      f.offset = 8;
      f.datatype = sensor_msgs::PointField::FLOAT32;
      f.count = 1;
      fds.emplace_back(f);

      return fds;
    };
    static const std::vector<sensor_msgs::PointField> fields = genFields();
    struct __attribute__((packed)) F {float x,y,z;};
    pc2.header.stamp.fromSec(steady_clock_now());
    pc2.header.frame_id = frame_id;
    pc2.height = 1;
    const int nbPt = (sgbmImage.width / 2) * (sgbmImage.height / 2);
    pc2.width = nbPt;
    pc2.fields = fields; /// xyz
    pc2.is_bigendian = false;
    pc2.point_step = 3*sizeof(float);
    pc2.row_step = pc2.point_step * pc2.width;
    pc2.is_dense = true;

    pc2.data.resize(nbPt*pc2.point_step);
    F* data = reinterpret_cast<F*>(pc2.data.data());

    short *dataTemp = (short *)sgbmImage.data.get();
    Vector3d p3d_in_sgbm;

    int index = 0;
    for(unsigned int i = 0; i < sgbmImage.height ; i = i + 2)
    {
        for(unsigned int j = 0; j < sgbmImage.width; j = j + 2, data++)
        {
            #ifdef SGBM_POINTCLOUD_UNIT_M
            index = (i * sgbmImage.width + j) * 3;
            p3d_in_sgbm = { (double(*(dataTemp + index)/1000.0)), 
                            double((*(dataTemp + index + 1))/1000.0), 
                            double((*(dataTemp + index + 2))/1000.0)};
            #else
            p3d_in_sgbm = { double(*dataTemp), 
                            double(*(dataTemp + 1)), 
                            double(*(dataTemp + 2))};
            #endif

            #ifdef CONVERT_TO_WORLD_COORDINATE_SYSTEM
            const Vector3d p3d_in_world = Transform(sgbm_pose_in_world.translation(), sgbm_pose_in_world.rotation()) * p3d_in_sgbm;
            if (p3dsInWorldFrame)
            {
                *data = F { float(p3d_in_world[0]), float(p3d_in_world[1]), float(p3d_in_world[2]) };
            }
            else
            {
                *data = F { float(p3d_in_sgbm[0]), float(p3d_in_sgbm[1]), float(p3d_in_sgbm[2]) };
            }
            #else
            *data = F { float(p3d_in_sgbm[0]), float(p3d_in_sgbm[1]), float(p3d_in_sgbm[2]) };
            #endif
        }
    }
}
xv_sdk::PoseStampedConfidence toRosPoseStampedConfidence(const Pose& xvPose, const std::string& frame_id)
{
    xv_sdk::PoseStampedConfidence ps;
    static double old_timeStamp = steady_clock_now();
    if (xvPose.hostTimestamp() < 0)
      std::cerr << "XVSDK-ROS-WRAPPER toRosPoseStamped() Error: negative Pose host-timestamp" << std::endl;
    try
    {
        double currentTimestamp = xvPose.hostTimestamp() > 0.1 ? xvPose.hostTimestamp() : 0.1;
        ps.poseMsg.header.stamp.fromSec(currentTimestamp);
        old_timeStamp = currentTimestamp;
    }
    catch(std::runtime_error& ex) 
    {
        ps.poseMsg.header.stamp.fromSec(old_timeStamp);
    }
    ps.poseMsg.header.frame_id = frame_id;

    ps.poseMsg.pose.position.x = xvPose.x();
    ps.poseMsg.pose.position.y = xvPose.y();
    ps.poseMsg.pose.position.z = xvPose.z();

    const auto quat = xvPose.quaternion(); /// [qx,qy,qz,qw]
    ps.poseMsg.pose.orientation.x = quat[0];
    ps.poseMsg.pose.orientation.y = quat[1];
    ps.poseMsg.pose.orientation.z = quat[2];
    ps.poseMsg.pose.orientation.w = quat[3];

    ps.confidence = xvPose.confidence();
    return ps;
}
} /// namespace xv
