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

#include <moveit/handeye_calibration_target/handeye_target_base.h>

namespace moveit_handeye_calibration
{
const std::string LOGNAME = "handeye_target_base";

const std::size_t CAMERA_MATRIX_VECTOR_DIMENSION = 9;  // 3x3 camera intrinsic matrix
const std::size_t CAMERA_MATRIX_WIDTH = 3;
const std::size_t CAMERA_MATRIX_HEIGHT = 3;
const std::size_t CAMERA_DISTORTION_VECTOR_DIMISION = 5;  // distortion parameters (k1, k2, t1, t2, k3)

bool HandEyeTargetBase::setCameraIntrinsicParams(const sensor_msgs::CameraInfoPtr& msg)
{
  if (!msg)
  {
    ROS_ERROR_NAMED(LOGNAME, "CameraInfo msg is NULL.");
    return false;
  }

  if (msg->K.size() != CAMERA_MATRIX_VECTOR_DIMENSION)
  {
    ROS_ERROR_NAMED(LOGNAME, "Invalid camera matrix dimision, current is %ld, required is %zu.", msg->K.size(),
                    CAMERA_MATRIX_VECTOR_DIMENSION);
    return false;
  }

  if (msg->D.size() != CAMERA_DISTORTION_VECTOR_DIMISION)
  {
    ROS_ERROR_NAMED(LOGNAME, "Invalid distortion parameters dimision, current is %ld, required is %zu.", msg->D.size(),
                    CAMERA_DISTORTION_VECTOR_DIMISION);
    return false;
  }

  std::lock_guard<std::mutex> base_lock(base_mutex_);

  // Store camera matrix info
  for (size_t i = 0; i < CAMERA_MATRIX_WIDTH; i++)
  {
    for (size_t j = 0; j < CAMERA_MATRIX_HEIGHT; j++)
    {
      camera_matrix_.at<double>(i, j) = msg->K[i * CAMERA_MATRIX_WIDTH + j];
    }
  }

  // Store camera distortion info
  for (size_t i = 0; i < CAMERA_DISTORTION_VECTOR_DIMISION; i++)
  {
    distortion_coeffs_.at<double>(i, 0) = msg->D[i];
  }

  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Set camera intrinsic parameter to: " << *msg);
  return true;
}

}  // namespace moveit_handeye_calibration