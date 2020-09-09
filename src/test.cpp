/**
* testRosCameraReader.cpp: test file for CameraReader
* Author: Ravi Joshi
* Date: 2019/02/01
*/
 #include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>


void chatterCallback(const sensor_msgs::CompressedImagePtr & data)
{
  ros::Time now = ros::Time::now();
  ros::Time kinect2_pr2_stamp = data->header.stamp;
  std::cout << "Image topic lantency: " << now.toSec() - kinect2_pr2_stamp.toSec() << std::endl;

}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "camera_latency");
  ros::NodeHandle nh;

  // const std::string colorTopic = "/camera/color/image_raw";
  // const std::string camInfoTopic = "/camera/color/camera_info";
  // const std::string depthTopic = "/camera/aligned_depth_to_color/image_raw";

  const std::string colorTopic = "/kinect2/sd/image_depth_rect/compressed";
  const std::string camInfoTopic = "/kinect2/qhd/camera_info";
  ros::Subscriber sub = nh.subscribe(colorTopic, 1, chatterCallback);

  ros::spin();
  return 0;
}
