/**
* testRosCameraReader.cpp: test file for CameraReader
* Author: Ravi Joshi
* Date: 2019/02/01
*/

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ros_openpose/cameraReader.hpp>

typedef std::shared_ptr<ros_openpose::CameraReader> sPtrCameraReader;

void show(sPtrCameraReader readers)
{
  ros::Rate loopRate(10);
  while (ros::ok())
  {
    auto colorImage = readers->getColorFrame();
    auto depthImage = readers->getDepthFrame();

    // normalize the depth image and change the encoding type to 32float, to show the depth image
    cv::normalize(depthImage, depthImage, 1, 0, cv::NORM_MINMAX, CV_32F);

    if (!colorImage.empty())
      cv::imshow("color image", colorImage);
    else
      // display the error at most once per 10 seconds
      ROS_WARN_THROTTLE(10, "Empty color image frame detected. Ignoring...");

    if (!depthImage.empty())
      cv::imshow("depth image", depthImage);
    else
      // display the error at most once per 10 seconds
      ROS_WARN_THROTTLE(10, "Empty depth image frame detected. Ignoring...");

    int key = cv::waitKey(1) & 255;
    if (key == 27)  // escape key
      break;

    ros::spinOnce();
    loopRate.sleep();
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_camera_reader");
  ros::NodeHandle nh;

  // const std::string colorTopic = "/camera/color/image_raw";
  // const std::string camInfoTopic = "/camera/color/camera_info";
  // const std::string depthTopic = "/camera/aligned_depth_to_color/image_raw";

  const std::string colorTopic = "/kinect2/qhd/image_color_rect";
  const std::string camInfoTopic = "/kinect2/qhd/camera_info";
  const std::string depthTopic = "/kinect2/remapped/qhd/image_depth_rect";
  const auto cameraReader = std::make_shared<ros_openpose::CameraReader>(nh, colorTopic, depthTopic, camInfoTopic);

  show(cameraReader);

  return 0;
}
