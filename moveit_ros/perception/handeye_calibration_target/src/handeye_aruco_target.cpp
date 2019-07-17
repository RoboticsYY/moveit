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

#include <moveit/handeye_calibration_target/handeye_aruco_target.h>

namespace moveit_handeye_calibration
{
const std::string LOGNAME = "handeye_aruco_target";

std::vector<std::string> HandEyeArucoTarget::getDictionaryIds()
{
  std::vector<std::string> ids;
  for (const std::pair<const std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME>& name : dict_map_)
    ids.push_back(name.first);
  return ids;
}

bool HandEyeArucoTarget::setTargetIntrinsicParams(const int& markers_x, const int& markers_y, const int& marker_size,
                                                  const int& separation, const int& border_bits,
                                                  const std::string& dictionary_id)
{
  if (markers_x != markers_x_ || markers_y != markers_y_ || marker_size != marker_size_ || separation != separation_ ||
      border_bits != border_bits_ || dict_map_.find(dictionary_id)->second != dict_)
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Set target intrinsic params: "
                                        << "\n"
                                        << "markers_x_ " << std::to_string(markers_x) << "\n"
                                        << "markers_y_ " << std::to_string(markers_y) << "\n"
                                        << "marker_size " << std::to_string(marker_size) << "\n"
                                        << "separation " << std::to_string(separation) << "\n"
                                        << "border_bits" << std::to_string(border_bits) << "\n"
                                        << "dictionary_id " << dictionary_id << "\n");
  if (markers_x > 0 && markers_y > 0 && marker_size > 0 && separation > 0 && border_bits > 0 && !dictionary_id.empty())
  {
    std::lock_guard<std::mutex> lck(aruco_mtx_);
    markers_x_ = markers_x;
    markers_y_ = markers_y;
    marker_size_ = marker_size;
    separation_ = separation;
    border_bits_ = border_bits;

    const auto& it = dict_map_.find(dictionary_id);
    if (it != dict_map_.end())
      dict_ = it->second;
    else
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Invalid dictionary name.");
      return false;
    }
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Invalid target intrinsic params.");
    return false;
  }

  return true;
}

bool HandEyeArucoTarget::setTargetDimension(const double& marker_size, const double& marker_seperation)
{
  if (marker_size != marker_size_real_ || marker_seperation != marker_seperation_real_)
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Set target real dimensions: "
                                        << "\n"
                                        << "marker_size " << std::to_string(marker_size) << "\n"
                                        << "marker_seperation " << std::to_string(marker_seperation) << "\n");

  if (marker_size > 0 && marker_seperation > 0)
  {
    std::lock_guard<std::mutex> lck(aruco_mtx_);
    marker_size_real_ = marker_size;
    marker_seperation_real_ = marker_seperation;
    return true;
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Invalid target real dimensions.");
    return false;
  }
}

bool HandEyeArucoTarget::createTargetImage(cv::Mat& image)
{
  cv::Size image_size;
  image_size.width = markers_x_ * (marker_size_ + separation_) - separation_ + 2 * separation_;
  image_size.height = markers_y_ * (marker_size_ + separation_) - separation_ + 2 * separation_;

  try
  {
    // Create target
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dict_);
    cv::Ptr<cv::aruco::GridBoard> board =
        cv::aruco::GridBoard::create(markers_x_, markers_y_, float(marker_size_), float(separation_), dictionary);

    // Create target image
    board->draw(image_size, image, separation_, border_bits_);
  }
  catch (cv::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Aruco target image creation exception: " << e.what());
    return false;
  }

  return true;
}

bool HandEyeArucoTarget::detectTargetPose(cv::Mat& image)
{
  std::lock_guard<std::mutex> lck(base_mtx_);
  try
  {
    // Detect aruco board
    aruco_mtx_.lock();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dict_);
    cv::Ptr<cv::aruco::GridBoard> board =
        cv::aruco::GridBoard::create(markers_x_, markers_y_, marker_size_real_, marker_seperation_real_, dictionary);
    aruco_mtx_.unlock();
    cv::Ptr<cv::aruco::DetectorParameters> params_ptr(new cv::aruco::DetectorParameters());
#if CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION == 2
    params_ptr->doCornerRefinement = true;
#else
    params_ptr->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
#endif

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary, corners, ids, params_ptr);
    if (ids.empty())
    {
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "No aruco marker detected.");
      return false;
    }

    // Refine markers borders
    std::vector<std::vector<cv::Point2f>> rejected_corners;
    cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejected_corners, camera_matrix_, distor_coeffs_);

    // Estimate aruco board pose
    int valid = cv::aruco::estimatePoseBoard(corners, ids, board, camera_matrix_, distor_coeffs_, rvect_, tvect_);

    // Draw the markers and frame axis if at least one marker is detected
    if (valid == 0)
    {
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "Cannot estimate aruco target pose.");
      return false;
    }

    if (std::log10(std::fabs(rvect_[0])) > 10 || std::log10(std::fabs(rvect_[0])) > 10 ||
        std::log10(std::fabs(rvect_[0])) > 10 || std::log10(std::fabs(tvect_[0])) > 10 ||
        std::log10(std::fabs(tvect_[0])) > 10 || std::log10(std::fabs(tvect_[0])) > 10)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Invalid target pose, please check CameraInfo msg.");
    }

    cv::Mat image_color;
    cv::cvtColor(image, image_color, cv::COLOR_GRAY2RGB);
    cv::aruco::drawDetectedMarkers(image_color, corners);
    cv::aruco::drawAxis(image_color, camera_matrix_, distor_coeffs_, rvect_, tvect_, 0.1);
    image = image_color;
  }
  catch (const cv::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Aruco target detection exception: " << e.what());
    return false;
  }
  return true;
}

geometry_msgs::TransformStamped HandEyeArucoTarget::getPose(std::string& frame_id)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = "handeye_target";

  tf2::Quaternion q(0, 0, 0, 1);
  getTFQuaternion(q);
  transform_stamped.transform.rotation.x = q.getX();
  transform_stamped.transform.rotation.y = q.getY();
  transform_stamped.transform.rotation.z = q.getZ();
  transform_stamped.transform.rotation.w = q.getW();
  std::vector<double> t(3, 0);
  getTranslationVect(t);
  transform_stamped.transform.translation.x = t[0];
  transform_stamped.transform.translation.y = t[1];
  transform_stamped.transform.translation.z = t[2];

  return transform_stamped;
}

void HandEyeArucoTarget::getTFQuaternion(tf2::Quaternion& q)
{
  cv::Mat rm;
  cv::Rodrigues(rvect_, rm);
  if (rm.rows == 3 && rm.cols == 3)
  {
    tf2::Matrix3x3 m;
    m.setValue(rm.ptr<double>(0)[0], rm.ptr<double>(0)[1], rm.ptr<double>(0)[2], rm.ptr<double>(1)[0],
               rm.ptr<double>(1)[1], rm.ptr<double>(1)[2], rm.ptr<double>(2)[0], rm.ptr<double>(2)[1],
               rm.ptr<double>(2)[2]);
    m.getRotation(q);
  }
  else
  {
    ROS_ERROR_NAMED(LOGNAME, "Wrong rotation matrix dimensions: %dx%d, it should be 3x3", rm.rows, rm.cols);
  }
}

void HandEyeArucoTarget::getTranslationVect(std::vector<double>& t)
{
  if (tvect_.rows == 3 && tvect_.cols == 1)
  {
    t.clear();
    t.resize(3);
    for (size_t i = 0; i < 3; ++i)
      t[i] = tvect_[i];
  }
  else
  {
    ROS_ERROR_NAMED(LOGNAME, "Wrong translation matrix dimensions: %dx%d, it should be 3x1", tvect_.rows, tvect_.cols);
  }
}

}  // namespace moveit_handeye_calibration
