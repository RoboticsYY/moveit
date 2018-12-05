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

#ifndef MOVEIT_MOVEIT_SETUP_ASSISTANT_WIDGETS_HANDEYE_CALIBRATION_WIDGET_
#define MOVEIT_MOVEIT_SETUP_ASSISTANT_WIDGETS_HANDEYE_CALIBRATION_WIDGET_

// Qt
#include <QMutex>
#include <QFrame>
#include <QObject>
#include <QWidget>
#include <QString>
#include <QComboBox>
#include <QGroupBox>
#include <QLineEdit>
#include <QTreeView>
#include <QPushButton>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QStandardItemModel>

// Ros
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <handeye/CalibrateHandEye.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/static_transform_broadcaster.h>

// SA
#ifndef Q_MOC_RUN
#include <moveit/setup_assistant/tools/moveit_config_data.h>
#include <moveit/setup_assistant/tools/calibration_board_detector.h>
#endif

#include "header_widget.h"
#include "setup_screen_widget.h"

namespace moveit_setup_assistant
{
// **********************************************************
// Custom qtwidget class for image topic combobox
// **********************************************************
class ImageTopicComboBox : public QComboBox
{
  Q_OBJECT
public:
  ImageTopicComboBox(QWidget* parent = 0) : QComboBox(parent)
  {
  }
  ~ImageTopicComboBox(){};

protected:
  void mousePressEvent(QMouseEvent* event);
};

// **********************************************************
// Custom qtwidget class for camera_info topic combobox
// **********************************************************
class CameraInfoTopicComboBox : public QComboBox
{
  Q_OBJECT
public:
  CameraInfoTopicComboBox(QWidget* parent = 0) : QComboBox(parent)
  {
  }
  ~CameraInfoTopicComboBox(){};

protected:
  void mousePressEvent(QMouseEvent* event);
};

// **********************************************************
// Custom qtwidget class for frame_name combobox
// **********************************************************
class TFFrameNameComboBox : public QComboBox
{
  Q_OBJECT
public:
  TFFrameNameComboBox(QWidget* parent = 0) : QComboBox(parent)
  {
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
  }
  ~TFFrameNameComboBox(){};

protected:
  void mousePressEvent(QMouseEvent* event);

private:
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener* tfListener_;
};

// ***********************************************************
// Custom qtwidget class for image view frame
// ***********************************************************
class ImageViewFrame : public QFrame
{
  Q_OBJECT
public:
  ImageViewFrame(QWidget* parent = 0, Qt::WindowFlags f = 0) : QFrame(parent, f)
  {
    connect(this, SIGNAL(update_view()), this, SLOT(update()), Qt::QueuedConnection);
  }
  ~ImageViewFrame(){};

  void drawImage(const QImage& qimage);

Q_SIGNALS:

  void update_view();

protected:
  void paintEvent(QPaintEvent* event);

private:
  QImage qimage_;
  QMutex image_mutex_;
};

// ******************************************************************************************
// User Interface for setting up the relative position and orientation of 3D sensors
// ******************************************************************************************
class HandeyeCalibrationWidget : public SetupScreenWidget
{
  Q_OBJECT
public:
  HandeyeCalibrationWidget(QWidget* parent, moveit_setup_assistant::MoveItConfigDataPtr config_data);

  /// Received when this widget is chosen from the navigation menu
  virtual void focusGiven();

  /// Received when another widget is chosen from the navigation menu
  virtual bool focusLost();

  /// Load handeye calibration configuration parameters
  void loadHandEyeCalibrationConfigParams();

  /// Called when the subscribed image message arrives
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// Called when the selected item in the calibration_board_type_field_ combobox is changed
  void calibrationBoardChanged(int index);

  /// Called when the selected item in the panel_selection_field_ combobox is changed
  void panelSelectionChanged(int index);

  /// Called when the take_sample_button_ is clicked
  void takeSampleButtonClicked(bool checked);

  /// Called when the reset_sample_button_ is clicked
  void resetSampleButtonClicked(bool checked);

  /// Called when the compute_button_ is clicked
  void solveButtonClicked(bool checked);

  /// Called when the publish_static_tf_button_ is clicked
  void publishButtonClicked(bool checked);

  /// Called when the item of image_topic_field_ combobox is selected
  void imageTopicComboboxChanged(int index);

  /// Called when the item of camera_info_topic_field_ combobox is selected
  void cameraInfoComboBoxChanged(int index);

private:
  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************

  // Group for calibration setting panel and calibrate operation panel
  QGroupBox* calibrate_setting_group_;
  QGroupBox* calibrate_operation_group_;
  QComboBox* panel_selection_field_;

  // Calibrate config panel
  QComboBox* sensor_mount_type_field_;
  QComboBox* calibration_solver_field_;
  ImageTopicComboBox* image_topic_field_;
  CameraInfoTopicComboBox* camera_info_topic_field_;
  QComboBox* calibration_board_type_field_;
  QLineEdit* square_size_field_;
  QLineEdit* marker_size_field_;
  QLineEdit* seperation_field_;

  TFFrameNameComboBox* sensor_frame_field_;
  TFFrameNameComboBox* target_frame_field_;
  TFFrameNameComboBox* end_effector_frame_field_;
  TFFrameNameComboBox* robot_base_frame_field_;

  // Calibrate operation panel
  QTreeView* sample_tree_view_;
  QStandardItemModel* tree_view_model_;
  QPushButton* take_sample_button_;
  QPushButton* reset_sample_button_;
  QPushButton* compute_button_;
  QPushButton* publish_static_tf_button_;
  ImageViewFrame* image_view_;

  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  /// Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;

  /// Stored data
  cv::Mat image_;

  /// Transform samples
  std::vector<geometry_msgs::Pose> effector_wrt_world_;
  std::vector<geometry_msgs::Pose> object_wrt_sensor_;

  /// Calibration result
  geometry_msgs::Pose camera_robot_;

  /// Calibration board detector
  moveit_setup_assistant::CalibrationBoardDetector calibration_board_detector_;

  // ******************************************************************************************
  // Ros Components
  // ******************************************************************************************

  /// ros node handler
  ros::NodeHandle nh_;

  /// ROS subscribers and publishers
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber camerainfo_sub_;

  /// tf listener
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener* tfListener_;

  /// tf boardcaster
  tf2_ros::TransformBroadcaster tfBroadcaster_;
  tf2_ros::StaticTransformBroadcaster tfStaticBroadcaster_;
};

}  // namespace moveit_setup_assistant

#endif
