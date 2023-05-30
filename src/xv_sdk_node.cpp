#include "xv_sdk/xv_sdk.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "xv_sdk");
  ros::NodeHandle nh("xv_sdk");

  ros::AsyncSpinner spinner(0); /// as many threads as procs to handle callbacks
  spinner.start();

  xv::RosWrapper wrapper(&nh);
  ROS_INFO("XV-SDK started");

  ros::waitForShutdown();
}
