/**
* rosOpenpose.cpp: the main file. it consists of two workers input and output worker.
*                  the job of the input worker is to provide color images to openpose wrapper.
*                  the job of the output worker is to receive the keypoints detected in 2D
*                  space. it then converts 2D pixels to 3D coordinates (wrt camera coordinate
*                  system)/
* Author: Ravi Joshi
* Date: 2019/09/27
* src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/tree/master/examples/tutorial_api_cpp
*/

// todo: merge the 'for' loop for body and hand keypoints into one

// OpenPose headers
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>

// ros_openpose headers
#include <ros_openpose/Frame.h>
#include <ros_openpose/cameraReader.hpp>

#include <std_msgs/Float64MultiArray.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  // Video write

// define a macro for compatibility with older versions
#define OPENPOSE1POINT6_OR_HIGHER OpenPose_VERSION_MAJOR >= 1 && OpenPose_VERSION_MINOR >= 6

// define sleep for input and output worker in milliseconds
const int SLEEP_MS = 10;

// define a few datatype
typedef std::shared_ptr<op::Datum> sPtrDatum;
typedef std::shared_ptr<std::vector<sPtrDatum>> sPtrVecSPtrDatum;


sPtrVecSPtrDatum createDatum(const std::shared_ptr<ros_openpose::CameraReader>& sPtrCameraReader)
{
  // get the latest color image from the camera
  auto& colorImage = sPtrCameraReader->getColorFrame();

  if (!colorImage.empty())
  {
    // create new datum
    auto datumToProcess = std::make_shared<std::vector<sPtrDatum>>();
    datumToProcess->emplace_back();
    auto& datumPtr = datumToProcess->at(0);
    datumPtr = std::make_shared<op::Datum>();

    // fill the datum
    #if OPENPOSE1POINT6_OR_HIGHER
              datumPtr->cvInputData = OP_CV2OPCONSTMAT(colorImage);
    #else
              datumPtr->cvInputData = colorImage;
    #endif
    return datumToProcess;
  }
  else
  {
      // display the error at most once per 10 seconds
      ROS_WARN_THROTTLE(10, "Empty color image frame detected. Ignoring...");
      return nullptr;
  }
}

ros_openpose::Frame pubHuman(const sPtrVecSPtrDatum& datumsPtr, const std::shared_ptr<ros_openpose::CameraReader>& sPtrCameraReader,
                  const ros::Publisher& mFramePublisher, const bool noDepth)
{
  ros_openpose::Frame mFrame;
  try
  {
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
      // update timestamp
      mFrame.header.stamp = ros::Time::now();

      // make sure to clear previous data
      mFrame.persons.clear();

      // we use the latest depth image for computing point in 3D space
      sPtrCameraReader->lockLatestDepthImage();

      // accesing each element of the keypoints
      const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
      const auto& handKeypoints = datumsPtr->at(0)->handKeypoints;

      // get the size
      const auto personCount = poseKeypoints.getSize(0);
      const auto bodyPartCount = poseKeypoints.getSize(1);
      const auto handPartCount = handKeypoints[0].getSize(1);

      mFrame.persons.resize(personCount);

      // update with the new data
      for (auto person = 0; person < personCount; person++)
      {
        mFrame.persons[person].bodyParts.resize(bodyPartCount);
        mFrame.persons[person].leftHandParts.resize(handPartCount);
        mFrame.persons[person].rightHandParts.resize(handPartCount);

        //   fillBodyROSMsg
        #pragma omp parallel for
          for (auto bodyPart = 0; bodyPart < bodyPartCount; bodyPart++)
          {
            // src:
            // https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md#keypoint-format-in-the-c-api
            const auto baseIndex = poseKeypoints.getSize(2) * (person * bodyPartCount + bodyPart);
            const auto x = poseKeypoints[baseIndex];
            const auto y = poseKeypoints[baseIndex + 1];
            const auto score = poseKeypoints[baseIndex + 2];

            float point3D[3];
            // compute 3D point only if depth flag is set
            if (!noDepth)
              sPtrCameraReader->computeMedium3DPoint(x, y, point3D);

            mFrame.persons[person].bodyParts[bodyPart].pixel.x = x;
            mFrame.persons[person].bodyParts[bodyPart].pixel.y = y;
            mFrame.persons[person].bodyParts[bodyPart].score = score;
            mFrame.persons[person].bodyParts[bodyPart].point.x = point3D[0];
            mFrame.persons[person].bodyParts[bodyPart].point.y = point3D[1];
            mFrame.persons[person].bodyParts[bodyPart].point.z = point3D[2];
          }
          //    fillHandROSMsg
       if(FLAGS_hand)
        {
          #pragma omp parallel for
              for (auto handPart = 0; handPart < handPartCount; handPart++)
              {
                const auto baseIndex = handKeypoints[0].getSize(2) * (person * handPartCount + handPart);

                // left hand
                const auto xLeft = handKeypoints[0][baseIndex];
                const auto yLeft = handKeypoints[0][baseIndex + 1];
                const auto scoreLeft = handKeypoints[0][baseIndex + 2];

                // right hand
                const auto xRight = handKeypoints[1][baseIndex];
                const auto yRight = handKeypoints[1][baseIndex + 1];
                const auto scoreRight = handKeypoints[1][baseIndex + 2];

                float point3DLeft[3];
                float point3DRight[3];

                // compute 3D point only if depth flag is set
                sPtrCameraReader->computeMedium3DPoint(xLeft, yLeft, point3DLeft);
                sPtrCameraReader->computeMedium3DPoint(xRight, yRight, point3DRight);

                mFrame.persons[person].leftHandParts[handPart].pixel.x = xLeft;
                mFrame.persons[person].leftHandParts[handPart].pixel.y = yLeft;
                mFrame.persons[person].leftHandParts[handPart].score = scoreLeft;
                mFrame.persons[person].leftHandParts[handPart].point.x = point3DLeft[0];
                mFrame.persons[person].leftHandParts[handPart].point.y = point3DLeft[1];
                mFrame.persons[person].leftHandParts[handPart].point.z = point3DLeft[2];

                mFrame.persons[person].rightHandParts[handPart].pixel.x = xRight;
                mFrame.persons[person].rightHandParts[handPart].pixel.y = yRight;
                mFrame.persons[person].rightHandParts[handPart].score = scoreRight;
                mFrame.persons[person].rightHandParts[handPart].point.x = point3DRight[0];
                mFrame.persons[person].rightHandParts[handPart].point.y = point3DRight[1];
                mFrame.persons[person].rightHandParts[handPart].point.z = point3DRight[2];
              }
        }
        }
      mFramePublisher.publish(mFrame);
    }
    else
    {
      // display the error at most once per 10 seconds
      ROS_WARN_THROTTLE(10, "Waiting for datum...");
      std::this_thread::sleep_for(std::chrono::milliseconds{SLEEP_MS});
    }
    return mFrame;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
  }
}

void pubPosition(const sPtrVecSPtrDatum& datumsPtr, const std::shared_ptr<ros_openpose::CameraReader>& sPtrCameraReader, const ros::Publisher& mposePublisher)
{
  try
  {
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
      // we use the latest depth image for computing point in 3D space
      sPtrCameraReader->lockLatestDepthImage();

      // accesing each element of the keypoints
      const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
      // const auto& handKeypoints = datumsPtr->at(0)->handKeypoints;

      // if there is a person
      if (poseKeypoints.getSize(0) > 0)
      {

        const auto bodyPartCount = poseKeypoints.getSize(1);
        // const auto handPartCount = handKeypoints[0].getSize(1);

        // only dectect the right wrist of the first person
        auto person = 0;
        std_msgs::Float64MultiArray handmsg;

        //record the right and shoudler center(1) position
        std::vector<int> bodyParts{4, 1};
        for (auto bodyPart: bodyParts)
        {
          // src:
          // https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md#keypoint-format-in-the-c-api
          const auto baseIndex = poseKeypoints.getSize(2) * (person * bodyPartCount + bodyPart);
          const auto x = poseKeypoints[baseIndex];
          const auto y = poseKeypoints[baseIndex + 1];

          float point3D[3];
          sPtrCameraReader->computeMedium3DPoint(x, y, point3D);

          // std::cout<< "right wrist, x: " << point3D[0] << "y: "<<  point3D[1] << "z: "<< point3D[2] <<std::endl;
          // std::cout<< "right wrist, pixel x: " << x << "y: "<<  y <<std::endl;
          for (auto point: point3D)
              handmsg.data.push_back(point);
        }

        // std::vector<int> handParts{0, 5, 9, 13, 17};
        // for (auto handPart: handParts)
        // {
        //   const auto baseIndex = handKeypoints[0].getSize(2) * (person * handPartCount + handPart);
        //
        //   // right hand
        //   const auto xRight = handKeypoints[1][baseIndex];
        //   const auto yRight = handKeypoints[1][baseIndex + 1];
        //
        //   float point3DRight[3];
        //
        //   // compute 3D point only if depth flag is set
        //   sPtrCameraReader->computeMedium3DPoint(xRight, yRight, point3DRight);
        //
        //   for (auto point: point3DRight)
        //       handmsg.data.push_back(point);
        // }
        mposePublisher.publish(handmsg);
      }
    }
    else
    {
      // display the error at most once per 10 seconds
      ROS_WARN_THROTTLE(10, "Waiting for datum...");
      std::this_thread::sleep_for(std::chrono::milliseconds{SLEEP_MS});
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
  }
}

void pubPosition_mframe(ros_openpose::Frame mFrame, const ros::Publisher& mposePublisher)
{
  std_msgs::Float64MultiArray handmsg;
  if (mFrame.persons.size() > 0)
  {
    std::vector<int> bodyParts{4, 1};
    for (auto bodyPart: bodyParts)
    {
      handmsg.data.push_back(mFrame.persons[0].bodyParts[bodyPart].point.x);
      handmsg.data.push_back(mFrame.persons[0].bodyParts[bodyPart].point.y);
      handmsg.data.push_back(mFrame.persons[0].bodyParts[bodyPart].point.z);
    }
    // std::vector<int> handParts{0, 5, 9, 13, 17};
    // for (auto handPart: handParts)
    // {
    //   handmsg.data.push_back(mFrame.persons[0].rightHandParts[handPart].point.x);
    //   handmsg.data.push_back(mFrame.persons[0].rightHandParts[handPart].point.y);
    //   handmsg.data.push_back(mFrame.persons[0].rightHandParts[handPart].point.z);
    // }
    mposePublisher.publish(handmsg);
  }
  else
  {
    // display the error at most once per 10 seconds
    ROS_WARN_THROTTLE(10, "Waiting for datum...");
  }
}

void printHumanKeypoints(const sPtrVecSPtrDatum& datumsPtr)
{
    // Example: How to use the pose keypoints
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        op::opLog("\nKeypoints:");
        // Accesing each element of the keypoints
        const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
        op::opLog("Person pose keypoints:");
        for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
        {
            op::opLog("Person " + std::to_string(person) + " (x, y, score):");
            for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
            {
                std::string valueToPrint;
                for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
                {
                    valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
                }
                op::opLog(valueToPrint);
            }
        }
        op::opLog(" ");
        // Alternative: just getting std::string equivalent
        if(FLAGS_face)
        {
            op::opLog("Face keypoints: " + datumsPtr->at(0)->faceKeypoints.toString(), op::Priority::High);
        }
        if(FLAGS_hand)
        {
            op::opLog("Left hand keypoints: " + datumsPtr->at(0)->handKeypoints[0].toString(), op::Priority::High);
            op::opLog("Right hand keypoints: " + datumsPtr->at(0)->handKeypoints[1].toString(), op::Priority::High);
        }
        // Heatmaps
        const auto& poseHeatMaps = datumsPtr->at(0)->poseHeatMaps;
        if (!poseHeatMaps.empty())
        {
            op::opLog("Pose heatmaps size: [" + std::to_string(poseHeatMaps.getSize(0)) + ", "
                    + std::to_string(poseHeatMaps.getSize(1)) + ", "
                    + std::to_string(poseHeatMaps.getSize(2)) + "]");
            const auto& faceHeatMaps = datumsPtr->at(0)->faceHeatMaps;
            op::opLog("Face heatmaps size: [" + std::to_string(faceHeatMaps.getSize(0)) + ", "
                    + std::to_string(faceHeatMaps.getSize(1)) + ", "
                    + std::to_string(faceHeatMaps.getSize(2)) + ", "
                    + std::to_string(faceHeatMaps.getSize(3)) + "]");
            const auto& handHeatMaps = datumsPtr->at(0)->handHeatMaps;
            op::opLog("Left hand heatmaps size: [" + std::to_string(handHeatMaps[0].getSize(0)) + ", "
                    + std::to_string(handHeatMaps[0].getSize(1)) + ", "
                    + std::to_string(handHeatMaps[0].getSize(2)) + ", "
                    + std::to_string(handHeatMaps[0].getSize(3)) + "]");
            op::opLog("Right hand heatmaps size: [" + std::to_string(handHeatMaps[1].getSize(0)) + ", "
                    + std::to_string(handHeatMaps[1].getSize(1)) + ", "
                    + std::to_string(handHeatMaps[1].getSize(2)) + ", "
                    + std::to_string(handHeatMaps[1].getSize(3)) + "]");
        }
    }
    else
        op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
}

// clang-format off
void configureOpenPose(op::Wrapper& opWrapper)
// clang-format on
 {
  try
  {
// Configuring OpenPose

// clang-format off
// logging_level
#if OPENPOSE1POINT6_OR_HIGHER
    op::checkBool(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255,
#else
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255,
#endif
              "Wrong logging_level value.",
              __LINE__,
              __FUNCTION__,
              __FILE__);

    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
    op::Profiler::setDefaultX(FLAGS_profile_speed);

// Applying user defined configuration - GFlags to program variables
// outputSize
#if OPENPOSE1POINT6_OR_HIGHER
    const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
#else
    const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
#endif

// netInputSize
#if OPENPOSE1POINT6_OR_HIGHER
    const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
#else
    const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
#endif

// faceNetInputSize
#if OPENPOSE1POINT6_OR_HIGHER
    const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
#else
    const auto faceNetInputSize = op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)");
#endif

// handNetInputSize
#if OPENPOSE1POINT6_OR_HIGHER
    const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
#else
    const auto handNetInputSize = op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)");
#endif

    // poseMode
    const auto poseMode = op::flagsToPoseMode(FLAGS_body);

// poseModel
#if OPENPOSE1POINT6_OR_HIGHER
    const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
#else
    const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
#endif

    // JSON saving
    if (!FLAGS_write_keypoint.empty())
      ROS_INFO("Flag `write_keypoint` is deprecated and will eventually be removed. Please, use `write_json` instead.");

    // keypointScaleMode
    const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);

    // heatmaps to add
    const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts,
                                                  FLAGS_heatmaps_add_bkg,
                                                  FLAGS_heatmaps_add_PAFs);

    const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);

    // >1 camera view?
    // const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1 || FLAGS_flir_camera);
    const auto multipleView = false;

    // Face and hand detectors
    const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
    const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);

    // Enabling Google Logging
    const bool enableGoogleLogging = true;

    // Initializing the user custom classes
    // auto wUserInput = std::make_shared<WUserInput>(cameraReader);
    // auto wUserOutput = std::make_shared<WUserOutput>(framePublisher, rightWristPublisher, cameraReader, frameId, noDepth, printKeypoints);
    //
    // // Add custom processing
    // const auto workerInputOnNewThread = true;
    // opWrapper.setWorker(op::WorkerType::Input, wUserInput, workerInputOnNewThread);
    //
    // const auto workerOutputOnNewThread = true;
    // opWrapper.setWorker(op::WorkerType::Output, wUserOutput, workerOutputOnNewThread);

    // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
    const op::WrapperStructPose wrapperStructPose{poseMode,
                                                  netInputSize,
                                                  outputSize,
                                                  keypointScaleMode,
                                                  FLAGS_num_gpu,
                                                  FLAGS_num_gpu_start,
                                                  FLAGS_scale_number,
                                                  (float)FLAGS_scale_gap,
                                                  op::flagsToRenderMode(FLAGS_render_pose,
                                                                        multipleView),
                                                  poseModel,
                                                  !FLAGS_disable_blending,
                                                  (float)FLAGS_alpha_pose,
                                                  (float)FLAGS_alpha_heatmap,
                                                  FLAGS_part_to_show,
#if OPENPOSE1POINT6_OR_HIGHER
                                                  op::String(FLAGS_model_folder),
#else
                                                  FLAGS_model_folder,
#endif
                                                  heatMapTypes,
                                                  heatMapScaleMode,
                                                  FLAGS_part_candidates,
                                                  (float)FLAGS_render_threshold,
                                                  FLAGS_number_people_max,
                                                  FLAGS_maximize_positives,
                                                  FLAGS_fps_max,
#if OPENPOSE1POINT6_OR_HIGHER
                                                  op::String(FLAGS_prototxt_path),
                                                  op::String(FLAGS_caffemodel_path),
#else
                                                  FLAGS_prototxt_path,
                                                  FLAGS_caffemodel_path,
#endif
                                                  (float)FLAGS_upsampling_ratio,
                                                  enableGoogleLogging};
    opWrapper.configure(wrapperStructPose);

    // Face configuration (use op::WrapperStructFace{} to disable it)
    const op::WrapperStructFace wrapperStructFace{FLAGS_face,
                                                  faceDetector,
                                                  faceNetInputSize,
                                                  op::flagsToRenderMode(FLAGS_face_render,
                                                                        multipleView,
                                                                        FLAGS_render_pose),
                                                  (float)FLAGS_face_alpha_pose,
                                                  (float)FLAGS_face_alpha_heatmap,
                                                  (float)FLAGS_face_render_threshold};
    opWrapper.configure(wrapperStructFace);

    // Hand configuration (use op::WrapperStructHand{} to disable it)
    const op::WrapperStructHand wrapperStructHand{FLAGS_hand,
                                                  handDetector,
                                                  handNetInputSize,
                                                  FLAGS_hand_scale_number,
                                                  (float)FLAGS_hand_scale_range,
                                                  op::flagsToRenderMode(FLAGS_hand_render,
                                                                        multipleView,
                                                                        FLAGS_render_pose),
                                                  (float)FLAGS_hand_alpha_pose,
                                                  (float)FLAGS_hand_alpha_heatmap,
                                                  (float)FLAGS_hand_render_threshold};
    opWrapper.configure(wrapperStructHand);

    // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
    const op::WrapperStructExtra wrapperStructExtra{FLAGS_3d,
                                                    FLAGS_3d_min_views,
                                                    FLAGS_identification,
                                                    FLAGS_tracking,
                                                    FLAGS_ik_threads};
    opWrapper.configure(wrapperStructExtra);

    // Output (comment or use default argument to disable any output)
    const op::WrapperStructOutput wrapperStructOutput{FLAGS_cli_verbose,
#if OPENPOSE1POINT6_OR_HIGHER
                                                      op::String(FLAGS_write_keypoint),
#else
                                                      FLAGS_write_keypoint,
#endif
                                                      op::stringToDataFormat(FLAGS_write_keypoint_format),
#if OPENPOSE1POINT6_OR_HIGHER
                                                      op::String(FLAGS_write_json),
                                                      op::String(FLAGS_write_coco_json),
#else
                                                      FLAGS_write_json,
                                                      FLAGS_write_coco_json,
#endif
                                                      FLAGS_write_coco_json_variants,
                                                      FLAGS_write_coco_json_variant,
#if OPENPOSE1POINT6_OR_HIGHER
                                                      op::String(FLAGS_write_images),
                                                      op::String(FLAGS_write_images_format),
                                                      op::String(FLAGS_write_video),
#else
                                                      FLAGS_write_images,
                                                      FLAGS_write_images_format,
                                                      FLAGS_write_video,
#endif
                                                      FLAGS_write_video_fps,
                                                      FLAGS_write_video_with_audio,
#if OPENPOSE1POINT6_OR_HIGHER
                                                      op::String(FLAGS_write_heatmaps),
                                                      op::String(FLAGS_write_heatmaps_format),
                                                      op::String(FLAGS_write_video_3d),
                                                      op::String(FLAGS_write_video_adam),
                                                      op::String(FLAGS_write_bvh),
                                                      op::String(FLAGS_udp_host),
                                                      op::String(FLAGS_udp_port)};
#else
                                                      FLAGS_write_heatmaps,
                                                      FLAGS_write_heatmaps_format,
                                                      FLAGS_write_video_3d,
                                                      FLAGS_write_video_adam,
                                                      FLAGS_write_bvh,
                                                      FLAGS_udp_host,
                                                      FLAGS_udp_port};
#endif
    opWrapper.configure(wrapperStructOutput);

    // GUI (comment or use default argument to disable any visual output)
    const op::WrapperStructGui wrapperStructGui{op::flagsToDisplayMode(FLAGS_display,
                                                                       FLAGS_3d),
                                                !FLAGS_no_gui_verbose,
                                                FLAGS_fullscreen};
    opWrapper.configure(wrapperStructGui);
    // clang-format on

    // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
    if (FLAGS_disable_multi_thread)
      opWrapper.disableMultiThreading();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ros_openpose_node");
  //  have to use multiple threads, otherwise the cameraReader thread cannot work
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::NodeHandle nh("~");

  // define the parameters, we are going to read
  bool noDepth, printKeypoints, pubTrackPoseMsg, pubSkeleton, pubTrackSpineMsg;
  std::string colorTopic, depthTopic, camInfoTopic, frameId, pubTopic;

  // read the parameters from relative nodel handle
  nh.getParam("frame_id", frameId);
  nh.getParam("pub_topic", pubTopic);
  nh.param("no_depth", noDepth, false);  // default value is false
  nh.getParam("color_topic", colorTopic);
  nh.getParam("depth_topic", depthTopic);
  nh.getParam("cam_info_topic", camInfoTopic);
  nh.param("print_keypoints", printKeypoints, false);
  nh.param("pub_RightWristMsg", pubTrackPoseMsg, false);
  nh.param("pub_SpineMsg", pubTrackSpineMsg, false);
  nh.param("skeleton", pubSkeleton, false);

  if (pubTopic.empty())
  {
    ROS_FATAL("Missing 'pub_topic' info in launch file. Please make sure that you have executed 'run.launch' file.");
    exit(-1);
  }

  // parsing command line flags
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const auto cameraReader = std::make_shared<ros_openpose::CameraReader>(nh, colorTopic, depthTopic, camInfoTopic);

  // the frame consists of the location of detected body parts of each person
  const ros::Publisher framePublisher = nh.advertise<ros_openpose::Frame>(pubTopic, 1);
  const ros::Publisher positionPublisher = nh.advertise<std_msgs::Float64MultiArray>("openpose_position", 1);

  // auto wUserOutput = std::make_shared<WUserOutput>(framePublisher, rightWristPublisher, cameraReader, frameId, noDepth);

  ROS_INFO("Starting ros_openpose...");
  op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
  configureOpenPose(opWrapper);

  // ros::Duration(1).sleep();
  // start and run
  opWrapper.start();

  while(ros::ok())
  {
    try
    {
      // ros::Time begin = ros::Time::now();

      auto frameNumber = cameraReader->getFrameNumber();
      // ros::Time image_kinect_time = cameraReader->getImageTime();
      // ros::Time get_iamge_time = ros::Time::now();
      // std::cout << "Image prepared to feed into : " << get_iamge_time.toSec() - image_kinect_time.toSec() << std::endl;
      if (frameNumber == 0 || frameNumber == 0ULL)
      {
        // display the error at most once per 10 seconds
        ROS_WARN_THROTTLE(10, "Waiting for color image frame...");
        std::this_thread::sleep_for(std::chrono::milliseconds{SLEEP_MS});
      }
      else
      {
        sPtrVecSPtrDatum datumToProcess = createDatum(cameraReader);

        bool successfullyEmplaced = opWrapper.waitAndEmplace(datumToProcess);

        sPtrVecSPtrDatum datumProcessed;
        if (successfullyEmplaced && opWrapper.waitAndPop(datumProcessed))
        {
              if(pubSkeleton)
              {
                ros_openpose::Frame mFrame = pubHuman(datumProcessed, cameraReader, framePublisher, noDepth);
                if (pubTrackPoseMsg && (!noDepth))
                  pubPosition_mframe(mFrame, rightWristPublisher);
              }
              else
              {
                if (pubTrackPoseMsg && (!noDepth))
                    pubPosition(datumProcessed, cameraReader, rightWristPublisher);
              }
              if (printKeypoints)
                printHumanKeypoints(datumProcessed);

          // ros::Time end = ros::Time::now();
          // std::cout << "Whole Openpose time " << end.toSec() - begin.toSec() << std::endl;
            // pubHuman needs 0.01s, in total 0.04/0.05s
            // if only pubPosition and no display, in total 0.034s
        }
        else
        {
            op::opLog("Processed datum could not be emplaced.", op::Priority::High,
                    __LINE__, __FUNCTION__, __FILE__);
        }
      }
    }
    catch (const std::exception& e)
    {
      // display the error at most once per 10 seconds
      ROS_ERROR_THROTTLE(10, "Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__,
                           __FILE__);
    }
  }

  // exit when Ctrl-C is pressed, or the node is shutdown by the master
  ros::waitForShutdown();

  // return successful message
  ROS_INFO("Exiting ros_openpose...");

  // stop processing
  opWrapper.stop();
  return 0;
}
