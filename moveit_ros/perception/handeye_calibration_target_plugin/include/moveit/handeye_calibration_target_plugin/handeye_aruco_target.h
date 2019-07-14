/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019,  Intel Corporation.
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
 *   * Neither the name of Willow Garage nor the names of its
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

#ifndef MOVEIT_HANDEYE_DEFAULT_TARGET_
#define MOVEIT_HANDEYE_DEFAULT_TARGET_

#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/handeye_calibration_target_plugin/handeye_target_base.h>

// opencv
#include <opencv2/aruco.hpp>

namespace moveit_handeye_calibration
{

class HandEyeArucoTarget : public HandEyeTargetBase
{
public:
  HandEyeArucoTarget() = default;
  ~HandEyeArucoTarget() = default;

  virtual bool initialize() override
  {
    // camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    // distor_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

    dict_map_ = {{"DICT_4X4_250", cv::aruco::DICT_4X4_250}, 
                 {"DICT_5X5_250", cv::aruco::DICT_5X5_250}, 
                 {"DICT_6X6_250", cv::aruco::DICT_6X6_250}, 
                 {"DICT_7X7_250", cv::aruco::DICT_7X7_250}, 
                 {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}};
    return true;
  }

  virtual bool setTargetIntrinsicParams(const int& markers_x, const int& markers_y, 
                                        const int& marker_size, const int& separation, 
                                        const int& border_bits, const std::string& dictionary_id) override;

  // virtual bool setCameraIntrinsicParams(const sensor_msgs::CameraInfoPtr& msg) override;

  virtual bool setTargetDimension(const double& marker_size, const double& marker_seperation) override;

  virtual bool createTargetImage(cv::Mat& image) override;

  virtual bool detectTargetPose(cv::Mat& image) override;

  virtual std::vector<std::string> getDictionaryIds() override;

  virtual geometry_msgs::TransformStamped getPose(std::string& frame_id) override;

protected:

  // Convert rvect_ to tf2::Quaternion 
  void getTFQuaternion(tf2::Quaternion& q);

  // Convert tvect_ to std::vector<double>
  void getTranslationVect(std::vector<double>& t);

private:

  // Predefined dictionary map
  std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dict_map_;

  // Target intrinsic params
  int markers_x_;       // Number of markers along X axis
  int markers_y_;       // Number of markers along Y axis
  int marker_size_;     // Marker size in pixels
  int separation_;      // Marker seperation distance in pixels
  int border_bits_;     // Margin of boarder in bits
  cv::aruco::PREDEFINED_DICTIONARY_NAME dict_; // Marker dictionary id
  
  // Target real dimensions in meters
  double marker_size_real_;         // Printed marker size
  double marker_seperation_real_;   // Printed marker seperation distance

  // Rotation and translation of the board w.r.t the camera frame
  cv::Vec3d tvect_, rvect_;
};

} // namespace moveit_handeye_calibration

#endif