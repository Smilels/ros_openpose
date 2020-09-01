/**
* CameraReader.hpp: header file for CameraReader
* Author: Ravi Joshi
* Date: 2019/02/01
*/

#pragma once

// ROS headers
#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

// CV brigge header
#include <cv_bridge/cv_bridge.h>

// OpenCV header
#include <opencv2/core/core.hpp>

// c++ headers
#include <mutex>
#include <vector>


// define a few datatype
typedef unsigned long long ullong;
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

namespace ros_openpose
{
  class CameraReader
  {
  private:
    cv::Mat mColorImage, mDepthImage;
    cv::Mat mColorImageUsed, mDepthImageUsed;
    std::string mColorTopic, mDepthTopic, mCamInfoTopic;
    std::mutex mMutex;
    ros::NodeHandle mNh;
    ros::Subscriber mCamInfoSubscriber;
    ros::Subscriber mColorImgSubscriber;
    ros::Subscriber mDepthImgSubscriber;

   // ros::Publisher handPublisher;

    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > mColorImgSubscriber_filter;
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > mDepthImgSubscriber_filter;
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > sync;

    ullong mFrameNumber = 0ULL;

    // camera calibration parameters
    std::shared_ptr<sensor_msgs::CameraInfo> mSPtrCameraInfo;

    inline void subscribe();
    void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& camMsg);
    void colorImgCallback(const sensor_msgs::ImageConstPtr& colorMsg);
    void depthImgCallback(const sensor_msgs::ImageConstPtr& depthMsg);
    void callback(const sensor_msgs::ImageConstPtr& colorMsg, const sensor_msgs::ImageConstPtr& depthMsg);


  public:
    // we don't want to instantiate using deafult constructor
    CameraReader() = delete;

    // copy constructor
    CameraReader(const CameraReader& other);

    // copy assignment operator
    CameraReader& operator=(const CameraReader& other);

    // main constructor
    CameraReader(ros::NodeHandle& nh, const std::string& colorTopic, const std::string& depthTopic,
                 const std::string& camInfoTopic);

    // we are okay with default destructor
    ~CameraReader() = default;

    // returns the current frame number
    // the frame number starts from 0 and increments
    // by 1 each time a frame (color) is received
    ullong getFrameNumber()
    {
      return mFrameNumber;
    }

    // returns the latest color frame from camera
    // it locks the color frame. remember that we
    // are just passing the pointer instead of copying whole data
    const cv::Mat& getColorFrame()
    {
      mMutex.lock();
      mColorImageUsed = mColorImage;
      mMutex.unlock();
      return mColorImageUsed;
    }

    // returns the latest depth frame from camera
    // it locks the depth frame. remember that we
    // are just passing the pointer instead of copying whole data
    const cv::Mat& getDepthFrame()
    {
      mMutex.lock();
      mDepthImageUsed = mDepthImage;
      mMutex.unlock();

      return mDepthImageUsed;
    }

    // lock the latest depth frame from camera. remember that we
    // are just passing the pointer instead of copying whole data
    void lockLatestDepthImage()
    {
      mMutex.lock();
      mDepthImageUsed = mDepthImage;
      mMutex.unlock();
    }

    // compute the point in 3D space for a given pixel without considering distortion
    void computeMedium3DPoint(const float pixelX, const float pixelY, float (&point)[3])
    {
      // K.at(0) = intrinsic.fx
      // K.at(4) = intrinsic.fy
      // K.at(2) = intrinsic.ppx
      // K.at(5) = intrinsic.ppy
      std::vector<float> points;
      float depth_;

      for (int v = -50; v < 50; ++v)
      {
          for (register int u = -50; u < 50; ++u)
          {
            auto depth = mDepthImageUsed.at<unsigned short>(static_cast<int>(pixelY + v), static_cast<int>(pixelX + u));
            depth_= (mDepthImageUsed.type() == 2) ? depth * 0.001f : depth;
            if (depth_ <= 0 || depth_ >2)
              continue;
            points.push_back(depth_);
          }
      }

      if (!points.empty()){
        if (points.size() % 2 == 0) {
            const auto median_it1 = points.begin() + points.size() / 2 - 1;
            const auto median_it2 = points.begin() + points.size() / 2;

            std::nth_element(points.begin(), median_it1 , points.end());
            const auto e1 = *median_it1;

            std::nth_element(points.begin(), median_it2 , points.end());
            const auto e2 = *median_it2;

            depth_ = (e1 + e2) / 2;

        } else {
            const auto median_it = points.begin() + points.size() / 2;
            std::nth_element(points.begin(), median_it , points.end());
            depth_ =  *median_it;
        }
      }

      auto x = (pixelX - mSPtrCameraInfo->K.at(2)) / mSPtrCameraInfo->K.at(0);
      auto y = (pixelY - mSPtrCameraInfo->K.at(5)) / mSPtrCameraInfo->K.at(4);
      point[0] = depth_ * x;
      point[1] = depth_ * y;
      point[2] = depth_;
    }

    // compute the point in 3D space for a given pixel without considering distortion
    void compute3DPoint(const float pixelX, const float pixelY, float (&point)[3])
    {
      // K.at(0) = intrinsic.fx
      // K.at(4) = intrinsic.fy
      // K.at(2) = intrinsic.ppx
      // K.at(5) = intrinsic.ppy

      // our depth frame type is 16UC1 which has unsigned short as an underlying type
      auto depth = mDepthImageUsed.at<unsigned short>(static_cast<int>(pixelY), static_cast<int>(pixelX));

      // 2 means CV_16UC1
      // we need to change depth to float, otherwise, the depth will go to a int 1 or 0
      float depth_;
      if (mDepthImageUsed.type() == 2)
          depth_ = depth * 0.001f;

      // no need to proceed further if the depth is zero or less than zero
      // the depth represents the distance of an object placed infront of the camera
      // therefore depth must always be a positive number
      if (depth_ <= 0)
        return;

      // the following calculation can also be done by image_geometry
      // for example:
      // image_geometry::PinholeCameraModel camModel;
      // camModel.fromCameraInfo(mSPtrCameraInfo);
      // cv::Point2d depthPixel(pixelX, pixelY);
      // auto point3d = camModel.projectPixelTo3dRay(depthPixel)
      // auto depth = mDepthImageUsed.at<unsigned short>(depthPixel);
      // point[0] = depth * point3d.x;
      // point[1] = depth * point3d.y;
      // point[2] = depth * point3d.z;
      // for more info., please see http://wiki.ros.org/image_geometry

      auto x = (pixelX - mSPtrCameraInfo->K.at(2)) / mSPtrCameraInfo->K.at(0);
      auto y = (pixelY - mSPtrCameraInfo->K.at(5)) / mSPtrCameraInfo->K.at(4);

      point[0] = depth_ * x;
      point[1] = depth_ * y;
      point[2] = depth_;
    }
  };
}
