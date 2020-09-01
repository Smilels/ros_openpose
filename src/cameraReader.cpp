/**
* cameraReader.cpp: class file for CameraReader. the CameraReader file provides
*                   functionality for retrieving color and depth images from a
*                   camera. it also reads the camera calibration parameters
*                   via ROS subscriber.
* Author: Ravi Joshi
* Date: 2019/02/01
*/

#include <ros_openpose/cameraReader.hpp>

namespace ros_openpose
{
  CameraReader::CameraReader(ros::NodeHandle& nh, const std::string& colorTopic, const std::string& depthTopic,
                             const std::string& camInfoTopic)
    : mNh(nh), mColorTopic(colorTopic), mDepthTopic(depthTopic), mCamInfoTopic(camInfoTopic)
  {
    // std::cout << "[" << this << "] constructor called" << std::endl;
    subscribe();
  }

  CameraReader::CameraReader(const CameraReader& other)
    : mNh(other.mNh), mColorTopic(other.mColorTopic), mDepthTopic(other.mDepthTopic), mCamInfoTopic(other.mCamInfoTopic)
  {
    // std::cout << "[" << this << "] copy constructor called" << std::endl;
    subscribe();
  }

  CameraReader& CameraReader::operator=(const CameraReader& other)
  {
    // std::cout << "[" << this << "] copy assignment called" << std::endl;
    mNh = other.mNh;
    mColorTopic = other.mColorTopic;
    mDepthTopic = other.mDepthTopic;
    mCamInfoTopic = other.mCamInfoTopic;

    subscribe();
    return *this;
  }

  // we define the subscriber here. we are using TimeSynchronizer filter to receive the synchronized data
  // we define the subscriber here. we are using TimeSynchronizer filter to receive the synchronized data
  inline void CameraReader::subscribe()
  {
    // handPublisher = mNh.advertise<std_msgs::Float64>("test", 1);

    mColorImgSubscriber_filter.reset( new message_filters::Subscriber<sensor_msgs::Image>( mNh, mColorTopic,1));
    mDepthImgSubscriber_filter.reset( new message_filters::Subscriber<sensor_msgs::Image>( mNh, mDepthTopic, 1));

    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    sync.reset(	new message_filters::Synchronizer<MySyncPolicy>( MySyncPolicy(10), *mColorImgSubscriber_filter, *mDepthImgSubscriber_filter));
    sync->registerCallback(boost::bind(&CameraReader::callback, this, _1, _2));

    // create a subscriber to read the camera parameters from the ROS
    mCamInfoSubscriber = mNh.subscribe<sensor_msgs::CameraInfo>(mCamInfoTopic, 1, &CameraReader::camInfoCallback, this);
  }

  void CameraReader::callback(const sensor_msgs::ImageConstPtr& colorMsg, const sensor_msgs::ImageConstPtr& depthMsg)
  {
    try
    {
      auto colorPtr = cv_bridge::toCvCopy(colorMsg, sensor_msgs::image_encodings::BGR8);

      auto depthPtr = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::TYPE_16UC1);

      // it is very important to lock the below assignment operation.
      // remember that we are accessing it from another thread too.
      std::lock_guard<std::mutex> lock(mMutex);
      mColorImage = colorPtr->image;
      mDepthImage = depthPtr->image;

      mFrameNumber++;
    }
    catch (cv_bridge::Exception& e)
    {
      // display the error at most once per 10 seconds
      ROS_ERROR_THROTTLE(10, "cv_bridge exception %s at line number %d on function %s in file %s", e.what(), __LINE__,
                         __FUNCTION__, __FILE__);
    }
  }

  void CameraReader::colorImgCallback(const sensor_msgs::ImageConstPtr& colorMsg)
  {
    try
    {
      auto colorPtr = cv_bridge::toCvCopy(colorMsg, sensor_msgs::image_encodings::BGR8);

      // it is very important to lock the below assignment operation.
      // remember that we are accessing it from another thread too.
      std::lock_guard<std::mutex> lock(mMutex);
      mColorImage = colorPtr->image;
      mFrameNumber++;
    }
    catch (cv_bridge::Exception& e)
    {
      // display the error at most once per 10 seconds
      ROS_ERROR_THROTTLE(10, "cv_bridge exception %s at line number %d on function %s in file %s", e.what(), __LINE__,
                         __FUNCTION__, __FILE__);
    }
  }

  void CameraReader::depthImgCallback(const sensor_msgs::ImageConstPtr& depthMsg)
  {
    try
    {
      auto depthPtr = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::TYPE_16UC1);

      // it is very important to lock the below assignment operation.
      // remember that we are accessing it from another thread too.
      std::lock_guard<std::mutex> lock(mMutex);

      // i assumed that by using cv_bridge::toCvCopy(msg, image_encodings::TYPE_16UC1)
      // function will change the encoding as well as convert the image format
      // however I found that it is only changing encoding.
      // therefore in case of '16UC1' encoding (depth values are in millimeter),
      // a manually conversion from millimeter to meter is required.

      mDepthImage = depthPtr->image;

      // in kinetic, multiply * 0.001f cannot work, it will output 0 or 1
      // (depthMsg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) ?
      //                   depthPtr->image * 0.001f : // convert to meter (SI units)
      //                   depthPtr->image; // no conversion needed
    }
    catch (cv_bridge::Exception& e)
    {
      // display the error at most once per 10 seconds
      ROS_ERROR_THROTTLE(10, "cv_bridge exception %s at line number %d on function %s in file %s", e.what(), __LINE__,
                         __FUNCTION__, __FILE__);
    }
  }

  void CameraReader::camInfoCallback(const sensor_msgs::CameraInfoConstPtr& camMsg)
  {
    mSPtrCameraInfo = std::make_shared<sensor_msgs::CameraInfo>(*camMsg);

    // since the calibration parameters are static so we don't need to keep running
    // the subscriber. that is why, we stop the subscriber once we receive
    // the parameters successfully
    if (mSPtrCameraInfo != nullptr)
      mCamInfoSubscriber.shutdown();
  }
}
