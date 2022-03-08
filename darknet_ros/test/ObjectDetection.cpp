/*
 * ObjectDetection.cpp
 *
 *  Created on: Jan 07, 2017
 *      Author: Marko Bjelonic
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */

// Google Test
#include <gtest/gtest.h>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// boost
#include <boost/thread.hpp>

// OpenCV2.
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Actions.
#include <darknet_ros_msgs/CheckForObjectsAction.h>

using CheckForObjectsActionClient = actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction>;
using CheckForObjectsActionClientPtr = std::shared_ptr<CheckForObjectsActionClient>;

// c++
#include <cmath>
#include <string>

darknet_ros_msgs::BoundingBoxes boundingBoxesResults_;

/*!
 * Done-callback for CheckForObjects action client.
 * @param[in] state
 * @param[in] result
 */
void checkForObjectsResultCB(const actionlib::SimpleClientGoalState& state, const darknet_ros_msgs::CheckForObjectsResultConstPtr& result) {
  std::cout << "[ObjectDetectionTest] Received bounding boxes." << std::endl;

  boundingBoxesResults_ = result->bounding_boxes;
}

bool sendImageToYolo(ros::NodeHandle nh, const std::string& pathToTestImage) {
  //! Check for objects action client.
  CheckForObjectsActionClientPtr checkForObjectsActionClient;

  // Action clients.
  std::string checkForObjectsActionName;
  nh.param("/darknet_ros/camera_action", checkForObjectsActionName, std::string("/darknet_ros/check_for_objects"));
  checkForObjectsActionClient.reset(new CheckForObjectsActionClient(nh, checkForObjectsActionName, true));

  // Wait till action server launches.
  if (!checkForObjectsActionClient->waitForServer(ros::Duration(20.0))) {
    std::cout << "[ObjectDetectionTest] sendImageToYolo(): checkForObjects action server has not been advertised." << std::endl;
    return false;
  }

  // Get test image
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  cv_ptr->image = cv::imread(pathToTestImage, CV_LOAD_IMAGE_COLOR);
  cv_ptr->encoding = sensor_msgs::image_encodings::RGB8;
  sensor_msgs::ImagePtr image = cv_ptr->toImageMsg();

  // Generate goal.
  darknet_ros_msgs::CheckForObjectsGoal goal;
  goal.image = *image;

  // Send goal.
  ros::Time beginYolo = ros::Time::now();
  checkForObjectsActionClient->sendGoal(goal, boost::bind(&checkForObjectsResultCB, _1, _2),
                                        CheckForObjectsActionClient::SimpleActiveCallback(),
                                        CheckForObjectsActionClient::SimpleFeedbackCallback());

  if (!checkForObjectsActionClient->waitForResult(ros::Duration(100.0))) {
    std::cout << "[ObjectDetectionTest] sendImageToYolo(): checkForObjects action server took to long to send back result." << std::endl;
    return false;
  }
  ros::Time endYolo = ros::Time::now();
  std::cout << "[ObjectDetectionTest] Object detection for one image took " << endYolo - beginYolo << " seconds." << std::endl;
  return true;
}

TEST(ObjectDetection, DetectRedBuoy) {
  srand(static_cast<unsigned int>(time(nullptr)));
  ros::NodeHandle nodeHandle("~");

  // Path to test image.
  std::string pathToTestImage = ros::package::getPath("darknet_ros");
  pathToTestImage += "/test_images/";
  pathToTestImage += "red_buoy.png";

  // Send dog image to yolo.
  ASSERT_TRUE(sendImageToYolo(nodeHandle, pathToTestImage));
  ASSERT_TRUE(sendImageToYolo(nodeHandle, pathToTestImage));

  // Evaluate if yolo was able to detect the three objects: dog, bicycle and car.
  bool detectedBuoy = false;
  double centerErrorBuoy;

  for (auto& boundingBox : boundingBoxesResults_.bounding_boxes) {
    double xPosCenter = boundingBox.xmin + (boundingBox.xmax - boundingBox.xmin) * 0.5;
    double yPosCenter = boundingBox.ymin + (boundingBox.ymax - boundingBox.ymin) * 0.5;

    if (boundingBox.Class == "mb_marker_buoy_red") {
      detectedBuoy = true;
      // std::cout << "centerErrorBuoy  " << xPosCenter << ", " <<  yPosCenter << std::endl;
      centerErrorBuoy = std::sqrt(std::pow(xPosCenter - 640, 2) + std::pow(yPosCenter - 310, 2));
    }
  }

  ASSERT_TRUE(detectedBuoy);
  EXPECT_LT(centerErrorBuoy, 40.0);
}
