/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Intel Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Intel nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Yu Yan */

#include <moveit/setup_assistant/tools/calibration_board_detector.h>

namespace moveit_setup_assistant
{

// *********************************************************************
// Set intrinsic camera parameters from sensor_msgs::CameraInfo message
// *********************************************************************
void CalibrationBoardDetector::setCameraMatrixFromMsg(const sensor_msgs::CameraInfo::ConstPtr & msg)
{
  if (msg)
  {    
    if (!msg->K.empty())
    {
      // Store camera matrix info
      cameraMatrix_.at<double>(0,0) = msg->K[0];
      cameraMatrix_.at<double>(0,1) = msg->K[1];
      cameraMatrix_.at<double>(0,2) = msg->K[2];
      cameraMatrix_.at<double>(1,0) = msg->K[3];
      cameraMatrix_.at<double>(1,1) = msg->K[4];
      cameraMatrix_.at<double>(1,2) = msg->K[5];
      cameraMatrix_.at<double>(2,0) = msg->K[6];
      cameraMatrix_.at<double>(2,1) = msg->K[7];
      cameraMatrix_.at<double>(2,2) = msg->K[8];
    }

    if (!msg->D.empty())
    {
      // Store camera distortion info
      distCoeffs_.at<double>(0, 0) = msg->D[0];
      distCoeffs_.at<double>(1, 0) = msg->D[1];
      distCoeffs_.at<double>(2, 0) = msg->D[2];
      distCoeffs_.at<double>(3, 0) = msg->D[3];
      distCoeffs_.at<double>(4, 0) = msg->D[4];
    }
  }
  else
  {
    ROS_ERROR_STREAM("Could not set camera intrinsic data, the message is NULL.");
  }
}

// ********************************************************************
// Convert rvect_ to tf2::Quaternion
// ********************************************************************
void CalibrationBoardDetector::CVRvec_to_ROSTFquaternion(tf2::Quaternion& q){
  cv::Mat rm;
  cv::Rodrigues(rvect_, rm);
  if (rm.rows == 3 && rm.cols == 3)
  {
    tf2::Matrix3x3 m;
    m.setValue(rm.ptr<double>(0)[0], rm.ptr<double>(0)[1], rm.ptr<double>(0)[2],
               rm.ptr<double>(1)[0], rm.ptr<double>(1)[1], rm.ptr<double>(1)[2],
               rm.ptr<double>(2)[0], rm.ptr<double>(2)[1], rm.ptr<double>(2)[2]);
    m.getRotation(q);
  }
  else
  {
    ROS_ERROR_STREAM("Transform from OpenCV rvec to ROS tf2 quaternion failed.");
  }
}

// ******************************************************************
// Get the tvect_ values
// ******************************************************************
void CalibrationBoardDetector::get_tvect(std::vector<double>& t)
{
  t.clear();
  t.resize(3);
  t[0] = tvect_[0];
  t[1] = tvect_[1];
  t[2] = tvect_[2];
}

// *******************************************************************
// Draw the coordinate frame axes of the detected calibration board
// *******************************************************************
void CalibrationBoardDetector::draw_axis(cv::Mat img, std::vector<cv::Point2f> corners, cv::Mat imgpts){
  if (!corners.empty())
  {
    cv::Point corner(corners[0]);
    if (imgpts.rows == 3 && imgpts.cols == 2)
    {
      cv::Point axis_point_x(imgpts.ptr<double>(0)[0], imgpts.ptr<double>(0)[1]);
      cv::Point axis_point_y(imgpts.ptr<double>(1)[0], imgpts.ptr<double>(1)[1]);
      cv::Point axis_point_z(imgpts.ptr<double>(2)[0], imgpts.ptr<double>(2)[1]);
      cv::line(img, corner, axis_point_x, cv::Scalar(255, 0, 0), 6);
      cv::line(img, corner, axis_point_y, cv::Scalar(0, 255, 0), 6);
      cv::line(img, corner, axis_point_z, cv::Scalar(0, 0, 255), 6);
    }
  }
}

// *****************************************************************
// Detect and get the pose and orientation of chessboard
// *****************************************************************
bool CalibrationBoardDetector::detectChessBoard(cv::Mat& image, int width, int height, double squareSize)
{
  // Find chessboard corners
  std::vector<cv::Point2f> pointBuf; //corners
  cv::Size patternsize;
  patternsize.width = width; patternsize.height = height;
  int chessBoardFlags;
  chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
  bool found = cv::findChessboardCorners(image, patternsize, pointBuf, chessBoardFlags);
  if (!found)
    return false;

  // Correct corner points
  cv::cornerSubPix(image, pointBuf, cv::Size(11,11), cv::Size(-1,-1), 
                   cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1 ));
  
  // Find the extrinsic transform between the calibration plate and camera plane
  std::vector<int> sizeOjbPnts = {static_cast<int>(pointBuf.size()), 3};
  cv::Mat objectPoints(sizeOjbPnts, CV_64F);
  for(size_t i=0; i < sizeOjbPnts[0]; i++)
  {
    objectPoints.at<double>(i, 0) = i%patternsize.width*squareSize;
    objectPoints.at<double>(i, 1) = i/patternsize.width*squareSize;
    objectPoints.at<double>(i, 2) = 0;
  }
  bool solved = cv::solvePnPRansac(objectPoints, pointBuf, cameraMatrix_, distCoeffs_, 
                                                                          rvect_, tvect_);
  if (!solved)
    return false;

  // Project the axis points
  cv::Mat axis = cv::Mat::zeros(3, 3, CV_64F);
  axis.at<double>(0, 0) = 3*squareSize; 
  axis.at<double>(1, 1) = 3*squareSize; 
  axis.at<double>(2, 2) = -3*squareSize;
  cv::Mat imageAxisPoints; 
  cv::projectPoints(axis, rvect_, tvect_, cameraMatrix_, distCoeffs_, imageAxisPoints);

  // Draw axis to image
  cv::Mat imageColor;
  cv::cvtColor(image, imageColor, cv::COLOR_GRAY2RGB);
  cv::drawChessboardCorners(imageColor, patternsize, cv::Mat(pointBuf), found);
  draw_axis(imageColor, pointBuf, imageAxisPoints);
  image = imageColor;
  return true;
}

// *************************************************************************
// Detect and get the pose and orientation of asymmetric circles grid
// *************************************************************************
bool CalibrationBoardDetector::detectAsymmetricCirclesGrid(cv::Mat& image, int width, int height, double gridSeperation)
{
  std::vector<cv::Point2f> pointBuf; //corners
  cv::Size patternsize;
  patternsize.width = width; patternsize.height = height;
  int chessBoardFlags;
  chessBoardFlags = cv::CALIB_CB_ASYMMETRIC_GRID;
  bool found = cv::findCirclesGrid(image, patternsize, pointBuf, chessBoardFlags);
  if (!found)
    return false;

  // Correct corner points
  std::vector<cv::Point2f> corners2;
  patternsize.height = (patternsize.height+1)/2;
  for(size_t i=0; i < patternsize.height; i++){
    for(size_t j=0; j < patternsize.width; j++)
      corners2.push_back(pointBuf[i*patternsize.width*2+j]);
  }
  pointBuf.clear();
  for(size_t i=0; i < corners2.size(); i++)
    pointBuf.push_back(corners2[i]);
  
  // Find the extrinsic transform between the calibration plate and camera plane
  std::vector<int> sizeOjbPnts = {static_cast<int>(pointBuf.size()), 3};
  cv::Mat objectPoints(sizeOjbPnts, CV_64F);
  for(size_t i=0; i < sizeOjbPnts[0]; i++){
    objectPoints.at<double>(i, 0) = i%patternsize.width*gridSeperation;
    objectPoints.at<double>(i, 1) = i/patternsize.width*gridSeperation;
    objectPoints.at<double>(i, 2) = 0;
  }
  bool solved = cv::solvePnPRansac(objectPoints, pointBuf, cameraMatrix_, distCoeffs_, 
                                                                          rvect_, tvect_);
  if (!solved)
    return false;

  // Project the axis points
  cv::Mat axis = cv::Mat::zeros(3, 3, CV_64F);
  axis.at<double>(0, 0) = 3 * gridSeperation; 
  axis.at<double>(1, 1) = 3 * gridSeperation; 
  axis.at<double>(2, 2) = -3 * gridSeperation;
  cv::Mat imageAxisPoints; 
  cv::projectPoints(axis, rvect_, tvect_, cameraMatrix_, distCoeffs_, imageAxisPoints);

  // Draw axis to image
  cv::Mat imageColor;
  cv::cvtColor(image, imageColor, cv::COLOR_GRAY2RGB);
  cv::drawChessboardCorners(imageColor, patternsize, cv::Mat(pointBuf), found);
  draw_axis(imageColor, pointBuf, imageAxisPoints);
  image = imageColor;
  return true;
}

// *************************************************************************
// Detect and get the pose and orientation of aruco board
// *************************************************************************
bool CalibrationBoardDetector::detectArucoBoard(cv::Mat& image, int width, int height, 
                                                cv::aruco::PREDEFINED_DICTIONARY_NAME dict, 
                                                double markerSize, double markerSeperation)
{
  // Detect aruco board
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dict);
  cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(width, height, markerSize, markerSeperation, dictionary);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(image, dictionary, corners, ids);
  if (ids.size() == 0)
    return false;

  // Refine markers borders
  std::vector<std::vector<cv::Point2f>> rejectedCorners;
  cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejectedCorners, 
                                                                  cameraMatrix_, distCoeffs_);
  // Estimate charuco board pose
  int valid = cv::aruco::estimatePoseBoard(corners, ids, board, 
                                                    cameraMatrix_, distCoeffs_, rvect_, tvect_);
  // Draw the markers and frame axis if at least one marker is detected
  if (valid == 0)
    return false;

  cv::Mat imageColor;
  cv::cvtColor(image, imageColor, cv::COLOR_GRAY2RGB);
  cv::aruco::drawDetectedMarkers(imageColor, corners, ids);
  cv::aruco::drawAxis(imageColor, cameraMatrix_, distCoeffs_, rvect_, tvect_, 0.1);
  image = imageColor;
  return true;
}

// *************************************************************************
// Detect and get the pose and orientation of charuco board
// *************************************************************************
bool CalibrationBoardDetector::detectCharucoBoard(cv::Mat& image, int width, int height,
                                                  cv::aruco::PREDEFINED_DICTIONARY_NAME dict,
                                                  double squareSize, double markerSize)
{
  // Detect ChArUco
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dict);
  cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(width, height, squareSize, markerSize, dictionary);
  cv::Ptr<cv::aruco::DetectorParameters> params_ptr(new cv::aruco::DetectorParameters());
  params_ptr->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(image, dictionary, corners, ids, params_ptr);
  if (ids.size() == 0)
    return false;

  // Refine markers borders
  std::vector<cv::Point2f> charucoCorners;
  std::vector<int> charucoIds; 
  cv::aruco::interpolateCornersCharuco(corners, ids, image, board, charucoCorners, charucoIds);
  if(charucoIds.size() == 0)
    return false;

  // Estimate charuco pose
  bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, 
                                                    cameraMatrix_, distCoeffs_, rvect_, tvect_);
  // Draw the markers and frame axis if at least one marker is detected                                                  
  if(!valid)
    return false;
  cv::Mat imageColor;
  cv::cvtColor(image, imageColor, cv::COLOR_GRAY2RGB);
  cv::aruco::drawDetectedCornersCharuco(imageColor, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
  cv::aruco::drawAxis(imageColor, cameraMatrix_, distCoeffs_, rvect_, tvect_, 0.1);
  image = imageColor;
  return true;
}

} /// namespace