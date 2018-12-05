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

#ifndef MOVEIT_MOVEIT_SETUP_ASSISTANT_TOOLS_CALIBRATION_BOARD_POSE_ESTIMATOR_
#define MOVEIT_MOVEIT_SETUP_ASSISTANT_TOOLS_CALIBRATION_BOARD_POSE_ESTIMATOR_

// OpenCV include
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <sensor_msgs/CameraInfo.h>

// ros include
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace moveit_setup_assistant
{

class CalibrationBoardDetector
{
public:

  CalibrationBoardDetector()
  {
    cameraMatrix_ = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs_ = cv::Mat::zeros(5, 1, CV_64F);
  }
  ~CalibrationBoardDetector(){}

  /// Set the camera intrinsic parameters
  void setCameraMatrixFromMsg(const sensor_msgs::CameraInfo::ConstPtr & msg);

  /// Detect the pose of calibration board
  bool detectChessBoard(cv::Mat& image, int width, int height, double squareSize);
  bool detectAsymmetricCirclesGrid(cv::Mat& image, int width, int height, double gridSeperation);
  bool detectArucoBoard(cv::Mat& image, int width, int height, 
                        cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary, 
                        double markerSize, double markerSeperation);
  bool detectCharucoBoard(cv::Mat& image, int width, int height,
                          cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary,
                          double squareSize, double markerSize);

  /// Convert rvect_ to tf2::Quaternion 
  void CVRvec_to_ROSTFquaternion(tf2::Quaternion& q);

  /// Get the tvect_
  void get_tvect(std::vector<double>& t);

  /// Draw 3D axis on 2D image
  void draw_axis(cv::Mat img, std::vector<cv::Point2f> corners, cv::Mat imgpts);

private:
  
  /// 3x3 floating-point camera matrix  A = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}
  cv::Mat cameraMatrix_;

  /// Vector of distortion coefficients  (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) of 4, 5, or 8 elements
  cv::Mat distCoeffs_;

  /// Rotation and translation of the board w.r.t the camera frame
  cv::Vec3d tvect_, rvect_;

};

} // namespace

#endif