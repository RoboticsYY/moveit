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

#include "handeye_calibration_widget.h"

// Qt
#include <QSet>
#include <QLabel>
#include <QPainter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QDoubleValidator>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// Constructor
// ******************************************************************************************
HandeyeCalibrationWidget::HandeyeCalibrationWidget(QWidget* parent,
                                                   moveit_setup_assistant::MoveItConfigDataPtr config_data)
  : SetupScreenWidget(parent), config_data_(config_data), it_(nh_)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();
  layout->setAlignment(Qt::AlignTop);

  // Top Header Area ------------------------------------------------
  HeaderWidget* header = new HeaderWidget(
      "Handeye calibration", "Configure the position and orientation of your 3D sensors to work with Moveit! ", this);
  layout->addWidget(header);

  // global setting panel -------------------------------------------------------------------------------
  global_setting_group_ = new QGroupBox("Global Setting", this);
  layout->addWidget(global_setting_group_);
  QVBoxLayout* global_setting_layout = new QVBoxLayout();
  global_setting_group_->setLayout(global_setting_layout);

  QFormLayout* global_setting_form_layout = new QFormLayout();
  global_setting_layout->addLayout(global_setting_form_layout);

  // Sensor mount type field
  sensor_mount_type_field_ = new QComboBox(this);
  sensor_mount_type_field_->addItem("Eye-to-Hand");
  sensor_mount_type_field_->addItem("Eye-in-Hand");
  connect(sensor_mount_type_field_, SIGNAL(activated(int)), this, SLOT(sensorMountTypeChanged(int)));
  global_setting_form_layout->addRow("Sensor mount type:", sensor_mount_type_field_);

  // Calibration solver field
  calibration_solver_field_ = new QComboBox(this);
  calibration_solver_field_->addItem("handeye");
  global_setting_form_layout->addRow("AX=XB solver:", calibration_solver_field_);

  // Sensor mount type image label
  sensor_mount_type_image_label_ = new QLabel(this);
  sensor_mount_type_image_label_->setAlignment(Qt::AlignCenter);
  global_setting_layout->addWidget(sensor_mount_type_image_label_);

  // Object detection setting panel ----------------------------------------------------------------------
  object_detection_setting_group_ = new QGroupBox("Object Detection Setting", this);
  layout->addWidget(object_detection_setting_group_);
  QVBoxLayout* object_detection_setting_layout = new QVBoxLayout();
  object_detection_setting_group_->setLayout(object_detection_setting_layout);

  QFormLayout* object_detection_setting_form_layout = new QFormLayout();
  object_detection_setting_layout->addLayout(object_detection_setting_form_layout);

    // Image topic
  image_topic_field_ = new ImageTopicComboBox(this);
  connect(image_topic_field_, SIGNAL(activated(int)), this, SLOT(imageTopicComboboxChanged(int)));
  object_detection_setting_form_layout->addRow("Image topic:", image_topic_field_);

  // Camera_info topic
  camera_info_topic_field_ = new CameraInfoTopicComboBox(this);
  connect(camera_info_topic_field_, SIGNAL(activated(int)), this, SLOT(cameraInfoComboBoxChanged(int)));
  object_detection_setting_form_layout->addRow("CameraInfo topic:", camera_info_topic_field_);

  // Calibration board type field
  calibration_board_type_field_ = new QComboBox(this);
  calibration_board_type_field_->clear();
  calibration_board_type_field_->addItem("Other");
  calibration_board_type_field_->addItem("CHESS_BOARD_9x6");
  calibration_board_type_field_->addItem("ASYMMETRIC_CIRCLES_GRID_4x11");
  calibration_board_type_field_->addItem("ASYMMETRIC_CIRCLES_GRID_3X5");
  calibration_board_type_field_->addItem("ARUCO_BOARD_5x7");
  calibration_board_type_field_->addItem("ARUCO_BOARD_3x4");
  calibration_board_type_field_->addItem("CHARUCO_BOARD_5x7");
  connect(calibration_board_type_field_, SIGNAL(currentIndexChanged(int)), this, SLOT(calibrationBoardChanged(int)));
  object_detection_setting_form_layout->addRow("Calibration board:", calibration_board_type_field_);

  // Calibration board param field
  QHBoxLayout* calibration_board_param_layout = new QHBoxLayout();

  // Square size
  calibration_board_param_layout->addWidget(new QLabel("Square Size:"));
  square_size_field_ = new QLineEdit(this);
  square_size_field_->setValidator(new QDoubleValidator(this));
  calibration_board_param_layout->addWidget(square_size_field_);

  // Marker size
  calibration_board_param_layout->addWidget(new QLabel("Marker Size:"));
  marker_size_field_ = new QLineEdit(this);
  marker_size_field_->setValidator(new QDoubleValidator(this));
  calibration_board_param_layout->addWidget(marker_size_field_);

  // Seperation
  calibration_board_param_layout->addWidget(new QLabel("Seperation:"));
  seperation_field_ = new QLineEdit(this);
  seperation_field_->setValidator(new QDoubleValidator(this));
  calibration_board_param_layout->addWidget(seperation_field_);
  object_detection_setting_form_layout->addRow(calibration_board_param_layout);

  // Frame name setting panel ---------------------------------------------------------------------------
  frame_name_setting_group_ = new QGroupBox("Frame Name Setting", this);
  layout->addWidget(frame_name_setting_group_);
  QFormLayout* frame_name_setting_form_layout = new QFormLayout();
  frame_name_setting_group_->setLayout(frame_name_setting_form_layout);

  // Sensor frame name
  sensor_frame_field_ = new TFFrameNameComboBox(this);
  frame_name_setting_form_layout->addRow("Sensor frame:", sensor_frame_field_);

  // Object frame name
  target_frame_field_ = new TFFrameNameComboBox(this);
  frame_name_setting_form_layout->addRow("Object frame:", target_frame_field_);

  // End-effector frame name
  end_effector_frame_field_ = new TFFrameNameComboBox(this);
  frame_name_setting_form_layout->addRow("End-effector frame:", end_effector_frame_field_);

  // Robot base frame
  robot_base_frame_field_ = new TFFrameNameComboBox(this);
  frame_name_setting_form_layout->addRow("Robot base frame:", robot_base_frame_field_);

  // Calibrate operation panel----------------------------------------------------
  calibrate_operation_group_ = new QGroupBox("Calibrate Operation", this);
  layout->addWidget(calibrate_operation_group_);
  QHBoxLayout* operation_layout = new QHBoxLayout();
  calibrate_operation_group_->setLayout(operation_layout);

  // Left layout
  QHBoxLayout* calibrate_operation_panel_left_layout = new QHBoxLayout();
  operation_layout->addLayout(calibrate_operation_panel_left_layout);
  // Sample tree view
  sample_tree_view_ = new QTreeView(this);
  sample_tree_view_->setMaximumWidth(360);
  sample_tree_view_->setAlternatingRowColors(true);
  tree_view_model_ = new QStandardItemModel(sample_tree_view_);
  sample_tree_view_->setModel(tree_view_model_);
  sample_tree_view_->setHeaderHidden(true);
  sample_tree_view_->setIndentation(10);
  calibrate_operation_panel_left_layout->addWidget(sample_tree_view_);

  // right layout
  QVBoxLayout* calibrate_operation_panel_right_layout = new QVBoxLayout();
  // calibrate_operation_panel_right_layout->setAlignment(Qt::AlignLeft);
  operation_layout->addLayout(calibrate_operation_panel_right_layout);
  // Control buttons
  QHBoxLayout* buttons_layout = new QHBoxLayout();
  buttons_layout->setAlignment(Qt::AlignLeft);
  take_sample_button_ = new QPushButton("Take Snapshot", this);
  take_sample_button_->setMinimumHeight(55);
  connect(take_sample_button_, SIGNAL(clicked(bool)), this, SLOT(takeSampleButtonClicked(bool)));
  reset_sample_button_ = new QPushButton("Reset", this);
  reset_sample_button_->setMinimumHeight(55);
  connect(reset_sample_button_, SIGNAL(clicked(bool)), this, SLOT(resetSampleButtonClicked(bool)));
  compute_button_ = new QPushButton("Compute", this);
  compute_button_->setMinimumHeight(55);
  compute_button_->setEnabled(false);
  connect(compute_button_, SIGNAL(clicked(bool)), this, SLOT(computeButtonClicked(bool)));
  publish_static_tf_button_ = new QPushButton("Publish", this);
  publish_static_tf_button_->setMinimumHeight(55);
  publish_static_tf_button_->setEnabled(false);
  connect(publish_static_tf_button_, SIGNAL(clicked(bool)), this, SLOT(publishButtonClicked(bool)));
  buttons_layout->addWidget(take_sample_button_);
  buttons_layout->addWidget(reset_sample_button_);
  buttons_layout->addWidget(compute_button_);
  buttons_layout->addWidget(publish_static_tf_button_);
  calibrate_operation_panel_right_layout->addLayout(buttons_layout);
  // Image view area
  image_view_ = new ImageViewFrame(this);
  calibrate_operation_panel_right_layout->addWidget(image_view_);

  QHBoxLayout* navigation_button_layout = new QHBoxLayout();
  layout->addLayout(navigation_button_layout);
  previous_button_ = new QPushButton("Previous", this);
  connect(previous_button_, SIGNAL(clicked(bool)), this, SLOT(previousButtonClicked(bool)));
  next_button_ = new QPushButton("Next", this);
  connect(next_button_, SIGNAL(clicked(bool)), this, SLOT(nextButtonClicked(bool)));
  navigation_button_layout->addWidget(previous_button_);
  navigation_button_layout->addWidget(next_button_);

  // Finish Layout ------------------------------------------------------
  this->setLayout(layout);

  index_ = 0;
  panel_array_.push_back(global_setting_group_);
  panel_array_.push_back(object_detection_setting_group_);
  panel_array_.push_back(frame_name_setting_group_);
  panel_array_.push_back(calibrate_operation_group_);

  // load tf listener
  tfListener_ = new tf2_ros::TransformListener(tfBuffer_);

};

// ******************************************************************************************
// Received when the selected option in sensor_mount_type_field_ is changed
// ******************************************************************************************
void HandeyeCalibrationWidget::sensorMountTypeChanged(int index)
{
  std::string image_path;
  if (index == 0)  // eye-to-hand mode
    image_path = "./resources/source/eye-to-hand.png";
  if (index == 1)  // eye-in-hand mode
    image_path = "./resources/source/eye-in-hand.png";

  QImage qimage;
  if (qimage.load(image_path.c_str()))
  {
    qimage = qimage.scaledToHeight(280, Qt::SmoothTransformation);
    sensor_mount_type_image_label_->setPixmap(QPixmap::fromImage(qimage));
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("handeye_calibration_widget", "FAILED TO LOAD " << image_path);
  }
}

// ******************************************************************************************
// Received when the selected option in calibration_board_type_field_ is changed
// ******************************************************************************************
void HandeyeCalibrationWidget::calibrationBoardChanged(int index)
{
  std::map<std::string, GenericParameter> config_params_map = config_data_->getHandEyeCalibarionConfig();
  switch (index)
  {
    case 1:  // CHESS_BOARD_9x6 selected
    {
      square_size_field_->setEnabled(true);
      marker_size_field_->setEnabled(false);
      seperation_field_->setEnabled(false);
      square_size_field_->setText(QString(config_params_map["chessboard_square_size"].getValue().c_str()));
      break;
    }
    case 2:  // ASYMMETRIC_CIRCLES_GRID_4x11 selected
    {
      square_size_field_->setEnabled(false);
      marker_size_field_->setEnabled(false);
      seperation_field_->setEnabled(true);
      seperation_field_->setText(QString(config_params_map["circle_grid_seperation"].getValue().c_str()));
      break;
    }
    case 3:  // ASYMMETRIC_CIRCLES_GRID_3x5 selected
    {
      square_size_field_->setEnabled(false);
      marker_size_field_->setEnabled(false);
      seperation_field_->setEnabled(true);
      seperation_field_->setText(QString(config_params_map["circle_grid_seperation"].getValue().c_str()));
      break;
    }
    case 4:  // ARUCO_BOARD_5x7 selected
    {
      square_size_field_->setEnabled(false);
      marker_size_field_->setEnabled(true);
      seperation_field_->setEnabled(true);
      marker_size_field_->setText(QString(config_params_map["aruco_marker_size"].getValue().c_str()));
      seperation_field_->setText(QString(config_params_map["aruco_marker_seperation"].getValue().c_str()));
      break;
    }
    case 5:  // ARUCO_BOARD_5x7 selected
    {
      square_size_field_->setEnabled(false);
      marker_size_field_->setEnabled(true);
      seperation_field_->setEnabled(true);
      marker_size_field_->setText(QString(config_params_map["aruco_marker_size"].getValue().c_str()));
      seperation_field_->setText(QString(config_params_map["aruco_marker_seperation"].getValue().c_str()));
      break;
    }
    case 6:  // CHARUCO_BOARD_3x4 selected
    {
      square_size_field_->setEnabled(true);
      marker_size_field_->setEnabled(true);
      seperation_field_->setEnabled(false);
      square_size_field_->setText(QString(config_params_map["charuco_square_size"].getValue().c_str()));
      marker_size_field_->setText(QString(config_params_map["charuco_marker_size"].getValue().c_str()));
      break;
    }
    default:  // Other calibration board used
    {
      square_size_field_->setEnabled(false);
      marker_size_field_->setEnabled(false);
      seperation_field_->setEnabled(false);
      break;
    }
  }
  return;
}

// ******************************************************************************************
// Received when take_sample_button_ is clicked
// ******************************************************************************************
void HandeyeCalibrationWidget::takeSampleButtonClicked(bool checked)
{
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::Pose cTo;
  geometry_msgs::Pose bTe;

  // Get the transform of the object w.r.t the camera
  try
  {
    transformStamped = tfBuffer_.lookupTransform(sensor_frame_field_->currentText().toStdString(),
                                                 target_frame_field_->currentText().toStdString(), ros::Time(0));
    cTo.position.x = transformStamped.transform.translation.x;
    cTo.position.y = transformStamped.transform.translation.y;
    cTo.position.z = transformStamped.transform.translation.z;
    cTo.orientation.x = transformStamped.transform.rotation.x;
    cTo.orientation.y = transformStamped.transform.rotation.y;
    cTo.orientation.z = transformStamped.transform.rotation.z;
    cTo.orientation.w = transformStamped.transform.rotation.w;
  }
  catch (tf2::TransformException& e)
  {
    ROS_WARN("%s", e.what());
    return;
  }

  // Get the transform of the end-effector w.r.t the robot base
  try
  {
    transformStamped = tfBuffer_.lookupTransform(robot_base_frame_field_->currentText().toStdString(),
                                                 end_effector_frame_field_->currentText().toStdString(), ros::Time(0));
    bTe.position.x = transformStamped.transform.translation.x;
    bTe.position.y = transformStamped.transform.translation.y;
    bTe.position.z = transformStamped.transform.translation.z;
    bTe.orientation.x = transformStamped.transform.rotation.x;
    bTe.orientation.y = transformStamped.transform.rotation.y;
    bTe.orientation.z = transformStamped.transform.rotation.z;
    bTe.orientation.w = transformStamped.transform.rotation.w;
  }
  catch (tf2::TransformException& e)
  {
    ROS_WARN("%s", e.what());
    return;
  }

  // save the pose samples
  effector_wrt_world_.push_back(bTe);
  object_wrt_sensor_.push_back(cTo);

  if (effector_wrt_world_.size() > 5)
    compute_button_->setEnabled(true);

  // add the sample to the tree-view
  std::string item_name = "Sample " + std::to_string(effector_wrt_world_.size());
  QStandardItem* parent = new QStandardItem(QString(item_name.c_str()));
  QStandardItem* child_1 = new QStandardItem(QString("bTe"));
  QStandardItem* child_2 = new QStandardItem(QString("cTo"));
  QStandardItem* child_1_1 = new QStandardItem(QString("T"));
  child_1_1->appendRow(new QStandardItem(QString(std::string("x: " + std::to_string(bTe.position.x)).c_str())));
  child_1_1->appendRow(new QStandardItem(QString(std::string("y: " + std::to_string(bTe.position.y)).c_str())));
  child_1_1->appendRow(new QStandardItem(QString(std::string("z: " + std::to_string(bTe.position.z)).c_str())));
  QStandardItem* child_1_2 = new QStandardItem(QString("R"));
  child_1_2->appendRow(new QStandardItem(QString(std::string("x: " + std::to_string(bTe.orientation.x)).c_str())));
  child_1_2->appendRow(new QStandardItem(QString(std::string("y: " + std::to_string(bTe.orientation.y)).c_str())));
  child_1_2->appendRow(new QStandardItem(QString(std::string("z: " + std::to_string(bTe.orientation.z)).c_str())));
  child_1_2->appendRow(new QStandardItem(QString(std::string("w: " + std::to_string(bTe.orientation.w)).c_str())));
  child_1->appendRow(child_1_1);
  child_1->appendRow(child_1_2);
  QStandardItem* child_2_1 = new QStandardItem(QString("T"));
  child_2_1->appendRow(new QStandardItem(QString(std::string("x: " + std::to_string(cTo.position.x)).c_str())));
  child_2_1->appendRow(new QStandardItem(QString(std::string("y: " + std::to_string(cTo.position.y)).c_str())));
  child_2_1->appendRow(new QStandardItem(QString(std::string("z: " + std::to_string(cTo.position.z)).c_str())));
  QStandardItem* child_2_2 = new QStandardItem(QString("R"));
  child_2_2->appendRow(new QStandardItem(QString(std::string("x: " + std::to_string(cTo.orientation.x)).c_str())));
  child_2_2->appendRow(new QStandardItem(QString(std::string("y: " + std::to_string(cTo.orientation.y)).c_str())));
  child_2_2->appendRow(new QStandardItem(QString(std::string("z: " + std::to_string(cTo.orientation.z)).c_str())));
  child_2_2->appendRow(new QStandardItem(QString(std::string("w: " + std::to_string(cTo.orientation.w)).c_str())));
  child_2->appendRow(child_2_1);
  child_2->appendRow(child_2_2);
  parent->appendRow(child_1);
  parent->appendRow(child_2);
  tree_view_model_->appendRow(parent);
}

// ******************************************************************************************
// Received when reset_sample_button_ is clicked
// ******************************************************************************************
void HandeyeCalibrationWidget::resetSampleButtonClicked(bool checked)
{
  effector_wrt_world_.clear();
  object_wrt_sensor_.clear();
  tree_view_model_->clear();
  compute_button_->setEnabled(false);
  publish_static_tf_button_->setEnabled(false);
}

// ******************************************************************************************
// Received when compute_button_ is clicked
// ******************************************************************************************
void HandeyeCalibrationWidget::computeButtonClicked(bool checked)
{
  // Check if the transform samples are not empty
  if (effector_wrt_world_.empty() || object_wrt_sensor_.empty())
  {
    QMessageBox::critical(this, "Pose Samples Empty", "Please take at least six pose samples.");
    return;
  }

  // Check if the sizes of the two transform sets equal
  if (effector_wrt_world_.size() != object_wrt_sensor_.size())
  {
    QMessageBox::critical(this, "Samples Size Missmatch", "The two pose sample sets have different size.");
    return;
  }

  ros::ServiceClient client = nh_.serviceClient<handeye::CalibrateHandEye>("handeye_calibration");
  handeye::CalibrateHandEye srv;
  if (sensor_mount_type_field_->currentText().toStdString() == std::string("Eye-to-Hand"))
  {
    srv.request.setup = std::string("Fixed");
  }
  else
  {
    srv.request.setup = std::string("Moving");
  }
  srv.request.solver = std::string("TsaiLenz1989");
  srv.request.effector_wrt_world.poses = effector_wrt_world_;
  srv.request.object_wrt_sensor.poses = object_wrt_sensor_;
  if (client.call(srv) && srv.response.success)
  {
    camera_robot_ = srv.response.sensor_frame;
    publish_static_tf_button_->setEnabled(true);
  }
  else
  {
    QString msg("Call solver ");
    msg += "<b>" + calibration_solver_field_->currentText() + "</b> failed. ";
    msg += "Please check if you have run:\n rosrun handeye handeye_server.py";
    QMessageBox::critical(this, "Call Solver Fail", msg);
  }
}

// ******************************************************************************************
// Received when publish_static_tf_button_ is clicked
// ******************************************************************************************
void HandeyeCalibrationWidget::publishButtonClicked(bool checked)
{
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  if (sensor_mount_type_field_->currentText().toStdString() == std::string("Eye-to-Hand"))
  {
    transformStamped.header.frame_id = robot_base_frame_field_->currentText().toStdString();
  }
  else
  {
    transformStamped.header.frame_id = end_effector_frame_field_->currentText().toStdString();
  }
  transformStamped.child_frame_id = sensor_frame_field_->currentText().toStdString();
  transformStamped.transform.translation.x = camera_robot_.position.x;
  transformStamped.transform.translation.y = camera_robot_.position.y;
  transformStamped.transform.translation.z = camera_robot_.position.z;
  transformStamped.transform.rotation.x = camera_robot_.orientation.x;
  transformStamped.transform.rotation.y = camera_robot_.orientation.y;
  transformStamped.transform.rotation.z = camera_robot_.orientation.z;
  transformStamped.transform.rotation.w = camera_robot_.orientation.w;

  tfStaticBroadcaster_.sendTransform(transformStamped);
}

// ******************************************************************************************
// Received when next_button_ is clicked
// ******************************************************************************************
void HandeyeCalibrationWidget::nextButtonClicked(bool checked)
{
  if (index_ == 0) // on the global setting panel
  {
    std::string path = ros::package::getPath(calibration_solver_field_->currentText().toStdString());

    if (path.empty())
    {
      QString msg("Please install <a href=\"https://github.com/crigroup/handeye\">");
      msg += calibration_solver_field_->currentText() + "</a> package for calibration.";
      QMessageBox::warning(this, "Package Installation", msg);
      return;
    }
  }

  if (index_ == 1) // on the object detection setting panel
  {
    if (calibration_board_type_field_->currentIndex() > 0) // Default calibartion board is chosen
    {
      if (image_topic_field_->currentText().isEmpty() || camera_info_topic_field_->currentText().isEmpty())
      {
        QString msg("For detecting the pose of <b>");
        msg += calibration_board_type_field_->currentText() + 
               "</b> board. Please start a ROS camera, select a RGB " + 
               "<a href=\"http://docs.ros.org/api/sensor_msgs/html/msg/Image.html\">Image</a> Topic and a RGB " +
               "<a href=\"http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html\">CameraInfo</a> Topic. " + 
               "Otherwise, if you are using your own object detection, choose <b>Other</b> calibration board.";
        QMessageBox::warning(this, "Select Necessary Topics", msg);
        return;
      }
    }
  }

  if (index_ == 2) // on the frame name setting panel
  {
    if (sensor_frame_field_->currentText().isEmpty() || 
        target_frame_field_->currentText().isEmpty() ||
        end_effector_frame_field_->currentText().isEmpty() || 
        robot_base_frame_field_->currentText().isEmpty())
    {
      QString msg("At least one of the four frame names has not been specified yet. "
                  "Handeye calibration needs two kinds of transforms: <ol>"
                  "<li>Transform from the <b>detected object frame</b> to the <b>sensor frame</b></li>"
                  "<li>Transform from the <b>end-effector frame</b> to the <b>robot base frame</b></li></ol>");
      QMessageBox::warning(this, "Specify Required Frame Names", msg);
      return;
    }
  }

  if (index_ < panel_array_.size()-1)
    index_++;

  previous_button_->setEnabled(true);
  if (index_ == panel_array_.size()-1)
    next_button_->setEnabled(false);

  for (int i = 0; i < panel_array_.size(); i++)
  {
    if (i == index_)
      panel_array_[i]->setVisible(true);
    else 
      panel_array_[i]->setVisible(false);
  }  
}

// ******************************************************************************************
// Received when _previous_button_ is clicked
// ******************************************************************************************
void HandeyeCalibrationWidget::previousButtonClicked(bool checked)
{
  if (index_ > 0)
    index_--;

  next_button_->setEnabled(true);  
  if (index_ == 0)
    previous_button_->setEnabled(false);

  for (int i = 0; i < panel_array_.size(); i++)
  {
    if (i == index_)
      panel_array_[i]->setVisible(true);
    else 
      panel_array_[i]->setVisible(false);
  }
}

// ******************************************************************************************
// Received when this widget is chosen from the navigation menu
// ******************************************************************************************
void HandeyeCalibrationWidget::focusGiven()
{  
  index_ = 0;
  global_setting_group_->setVisible(true);
  object_detection_setting_group_->setVisible(false);
  frame_name_setting_group_->setVisible(false);
  calibrate_operation_group_->setVisible(false);
  previous_button_->setEnabled(false);

  loadHandEyeCalibrationConfigParams();
}

// ******************************************************************************************
// Received when another widget is chosen from the navigation menu
// ******************************************************************************************
bool HandeyeCalibrationWidget::focusLost()
{
  config_data_->changes |= MoveItConfigData::HANDEYE_CALIBRATION;
  config_data_->addGenericParameterToHandEyeCalibrationConfig(
      "sensor_mount_type", sensor_mount_type_field_->currentText().trimmed().toStdString());
  config_data_->addGenericParameterToHandEyeCalibrationConfig(
      "calibration_solver", calibration_solver_field_->currentText().trimmed().toStdString());
  if (!image_topic_field_->currentText().isEmpty())
    config_data_->addGenericParameterToHandEyeCalibrationConfig(
        "image_topic", image_topic_field_->currentText().trimmed().toStdString());
  if (!camera_info_topic_field_->currentText().isEmpty())
    config_data_->addGenericParameterToHandEyeCalibrationConfig(
        "camera_info_topic", camera_info_topic_field_->currentText().trimmed().toStdString());
  switch (calibration_board_type_field_->currentIndex())
  {
    case 1:  // CHESS_BOARD_9x6 selected
    {
      config_data_->addGenericParameterToHandEyeCalibrationConfig("chessboard_square_size",
                                                                  square_size_field_->text().trimmed().toStdString());
      break;
    }
    case 2:  // ASYMMETRIC_CIRCLES_GRID_4x11 selected
    {
      config_data_->addGenericParameterToHandEyeCalibrationConfig("circle_grid_seperation",
                                                                  seperation_field_->text().trimmed().toStdString());
      break;
    }
    case 3:  // ASYMMETRIC_CIRCLES_GRID_3x5 selected
    {
      config_data_->addGenericParameterToHandEyeCalibrationConfig("circle_grid_seperation",
                                                                  seperation_field_->text().trimmed().toStdString());
      break;
    }
    case 4:  // ARUCO_BOARD_5x7 selected
    {
      config_data_->addGenericParameterToHandEyeCalibrationConfig("aruco_marker_size",
                                                                  marker_size_field_->text().trimmed().toStdString());
      config_data_->addGenericParameterToHandEyeCalibrationConfig("aruco_marker_seperation",
                                                                  seperation_field_->text().trimmed().toStdString());
      break;
    }
    case 5:  // ARUCO_BOARD_5x7 selected
    {
      config_data_->addGenericParameterToHandEyeCalibrationConfig("aruco_marker_size",
                                                                  marker_size_field_->text().trimmed().toStdString());
      config_data_->addGenericParameterToHandEyeCalibrationConfig("aruco_marker_seperation",
                                                                  seperation_field_->text().trimmed().toStdString());
      break;
    }
    case 6:  // CHARUCO_BOARD_3x4 selected
    {
      config_data_->addGenericParameterToHandEyeCalibrationConfig("charuco_square_size",
                                                                  square_size_field_->text().trimmed().toStdString());
      config_data_->addGenericParameterToHandEyeCalibrationConfig("charuco_marker_size",
                                                                  marker_size_field_->text().trimmed().toStdString());
      break;
    }
    default:  // Other calibration board used
      break;
  }
  if (!sensor_frame_field_->currentText().isEmpty())
    config_data_->addGenericParameterToHandEyeCalibrationConfig(
        "sensor_frame", sensor_frame_field_->currentText().trimmed().toStdString());
  if (!target_frame_field_->currentText().isEmpty())
    config_data_->addGenericParameterToHandEyeCalibrationConfig(
        "target_frame", target_frame_field_->currentText().trimmed().toStdString());
  if (!end_effector_frame_field_->currentText().isEmpty())
    config_data_->addGenericParameterToHandEyeCalibrationConfig(
        "end_effector_frame", end_effector_frame_field_->currentText().trimmed().toStdString());
  if (!robot_base_frame_field_->currentText().isEmpty())
    config_data_->addGenericParameterToHandEyeCalibrationConfig(
        "robot_base_frame", robot_base_frame_field_->currentText().trimmed().toStdString());
  std::string camera_robot_translation = std::to_string(camera_robot_.position.x) + " " +
                                         std::to_string(camera_robot_.position.y) + " " +
                                         std::to_string(camera_robot_.position.z);
  std::string camera_robot_rotation =
      std::to_string(camera_robot_.orientation.x) + " " + std::to_string(camera_robot_.orientation.y) + " " +
      std::to_string(camera_robot_.orientation.z) + " " + std::to_string(camera_robot_.orientation.w);
  config_data_->addGenericParameterToHandEyeCalibrationConfig("camera_robot_translation", camera_robot_translation);
  config_data_->addGenericParameterToHandEyeCalibrationConfig("camera_robot_rotation", camera_robot_rotation);
}

// ******************************************************************************************
// Load default setting parameters and initialize the ros subscribers and publishers
// ******************************************************************************************
void HandeyeCalibrationWidget::loadHandEyeCalibrationConfigParams()
{
  // Only load this combo box once
  static bool hasLoaded = false;
  if (hasLoaded)
    return;
  hasLoaded = true;

  // Load deafult config, or use the one in the config package if exists
  std::map<std::string, GenericParameter> config_params_map = config_data_->getHandEyeCalibarionConfig();
  if (config_params_map["sensor_mount_type"].getValue() == std::string("Eye-to-Hand"))
    sensor_mount_type_field_->setCurrentIndex(0);
  else
    sensor_mount_type_field_->setCurrentIndex(1);

  if (config_params_map["calibration_solver"].getValue() == std::string("crigroup/handeye"))
    calibration_solver_field_->setCurrentIndex(0);

  ros::master::V_TopicInfo ros_topic_vec;
  ros::master::getTopics(ros_topic_vec);
  for (auto it = ros_topic_vec.begin(); it != ros_topic_vec.end(); it++)
  {
    // If the default image topic exists, subscribe to it
    if (!config_params_map["image_topic"].getValue().compare(it->name.c_str()))
    {
      image_topic_field_->addItem(QString(config_params_map["image_topic"].getValue().c_str()));
      std::string topic = config_params_map["image_topic"].getValue();
      image_sub_.shutdown();
      try
      {
        image_sub_ = it_.subscribe(topic, 1, &HandeyeCalibrationWidget::imageCallback, this);
      }
      catch (image_transport::TransportLoadException& e)
      {
        ROS_WARN_STREAM("Subscribe to image topic: " << topic << " failed. " << e.what());
      }
    }

    // If the default image info topic exists, subscribe to it
    if (!config_params_map["camera_info_topic"].getValue().compare(it->name.c_str()))
    {
      camera_info_topic_field_->addItem(QString(config_params_map["camera_info_topic"].getValue().c_str()));
      std::string topic = config_params_map["camera_info_topic"].getValue();
      camerainfo_sub_.shutdown();
      try
      {
        camerainfo_sub_ = nh_.subscribe(topic, 1, &HandeyeCalibrationWidget::cameraInfoCallback, this);
      }
      catch (ros::Exception& e)
      {
        ROS_WARN_STREAM("Subscribe to camera info topic: " << topic << " failed. " << e.what());
      }
    }
  }

  if (config_params_map["calibration_board_type"].getValue() == std::string("CHESS_BOARD_9x6"))
  {
    calibration_board_type_field_->setCurrentIndex(1);
    square_size_field_->setText(QString(config_params_map["chessboard_square_size"].getValue().c_str()));
  }
  else if (config_params_map["calibration_board_type"].getValue() == std::string("ASYMMETRIC_CIRCLES_GRID_4x11"))
  {
    calibration_board_type_field_->setCurrentIndex(2);
    seperation_field_->setText(QString(config_params_map["circle_grid_seperation"].getValue().c_str()));
  }
  else if (config_params_map["calibration_board_type"].getValue() == std::string("ARUCO_BOARD_5x7"))
  {
    calibration_board_type_field_->setCurrentIndex(3);
    marker_size_field_->setText(QString(config_params_map["aruco_marker_size"].getValue().c_str()));
    seperation_field_->setText(QString(config_params_map["aruco_marker_seperation"].getValue().c_str()));
  }
  else if (config_params_map["calibration_board_type"].getValue() == std::string("CHARUCO_BOARD_5x7"))
  {
    calibration_board_type_field_->setCurrentIndex(4);
    square_size_field_->setText(QString(config_params_map["charuco_square_size"].getValue().c_str()));
    marker_size_field_->setText(QString(config_params_map["charuco_marker_size"].getValue().c_str()));
  }
  else
  {
    calibration_board_type_field_->setCurrentIndex(0);
  }

  std::vector<std::string> frame_names;
  tfBuffer_._getFrameStrings(frame_names);

  for (auto it = frame_names.begin(); it != frame_names.end(); it++)
  {
    // If the default frames exist add them to the frame name combobox
    if (!config_params_map["sensor_frame"].getValue().compare((*it).c_str()))
      sensor_frame_field_->addItem(QString(config_params_map["sensor_frame"].getValue().c_str()));
    if (!config_params_map["target_frame"].getValue().compare((*it).c_str()))
      target_frame_field_->addItem(QString(config_params_map["target_frame"].getValue().c_str()));
    if (!config_params_map["end_effector_frame"].getValue().compare((*it).c_str()))
      end_effector_frame_field_->addItem(QString(config_params_map["end_effector_frame"].getValue().c_str()));
    if (!config_params_map["robot_base_frame"].getValue().compare((*it).c_str()))
      robot_base_frame_field_->addItem(QString(config_params_map["robot_base_frame"].getValue().c_str()));
  }

  // Load the sensor mount type image
  std::string image_path;
  if (sensor_mount_type_field_->currentIndex() == 0)  // eye-to-hand mode
    image_path = "./resources/source/eye-to-hand.png";
  if (sensor_mount_type_field_->currentIndex() == 1)  // eye-in-hand mode
    image_path = "./resources/source/eye-in-hand.png";

  QImage qimage;
  if (qimage.load(image_path.c_str()))
  {
    qimage = qimage.scaledToHeight(280, Qt::SmoothTransformation);
    sensor_mount_type_image_label_->setPixmap(QPixmap::fromImage(qimage));
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("handeye_calibration_widget", "FAILED TO LOAD " << image_path);
  }

  // assign default value of camera-robot transform
  std::istringstream buf;
  std::istream_iterator<std::string> beg, end;
  std::vector<std::string> tokens;
  std::string camera_robot_translation = config_params_map["camera_robot_translation"].getValue();
  buf = std::istringstream(camera_robot_translation);
  beg = std::istream_iterator<std::string>(buf);
  tokens = std::vector<std::string>(beg, end);
  camera_robot_.position.x = std::stod(tokens[0]);
  camera_robot_.position.y = std::stod(tokens[1]);
  camera_robot_.position.z = std::stod(tokens[2]);

  std::string camera_robot_rotation = config_params_map["camera_robot_rotation"].getValue();
  buf = std::istringstream(camera_robot_rotation);
  beg = std::istream_iterator<std::string>(buf);
  tokens = std::vector<std::string>(beg, end);
  camera_robot_.orientation.x = std::stod(tokens[0]);
  camera_robot_.orientation.y = std::stod(tokens[1]);
  camera_robot_.orientation.z = std::stod(tokens[2]);
  camera_robot_.orientation.w = std::stod(tokens[3]);
}

// **************************************************************************************************
// Callback function for dealing with the image message received from the topic of image_topic_field_
// ***************************************************************************************************
void HandeyeCalibrationWidget::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (msg->data.empty())
  {
    ROS_ERROR_STREAM("The image msg of the topic " << image_topic_field_->currentText().toStdString() << " is empty.");
    return;
  }

  if (msg->header.frame_id.empty())
  {
    ROS_ERROR_STREAM("frame_id is not specified in the message of the topic "
                     << image_topic_field_->currentText().toStdString() << ", tf cannot be published.");
  }

  cv_bridge::CvImagePtr cv_ptr;
  QImage qimage;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    image_view_->drawImage(qimage);
    return;
  }
  image_ = cv_ptr->image;
  bool detected = false;
  switch (calibration_board_type_field_->currentIndex())
  {
    case 1:  // CHESS_BOARD_9x6 selected
    {
      detected = calibration_board_detector_.detectChessBoard(image_, 9, 6, square_size_field_->text().toDouble());
      break;
    }
    case 2:  // ASYMMETRIC_CIRCLES_GRID_4x11 selected
    {
      detected = calibration_board_detector_.detectAsymmetricCirclesGrid(image_, 4, 11, seperation_field_->text().toDouble());
      break;
    }
    case 3:  // ASYMMETRIC_CIRCLES_GRID_3x5 selected
    {
      detected = calibration_board_detector_.detectAsymmetricCirclesGrid(image_, 3, 5, seperation_field_->text().toDouble());
      break;
    }
    case 4:  // ARUCO_BOARD_5x7 selected
    {
      detected = calibration_board_detector_.detectArucoBoard(image_, 5, 7, cv::aruco::DICT_6X6_250,
                                                              marker_size_field_->text().toDouble(),
                                                              seperation_field_->text().toDouble());
      break;
    }
    case 5:  // ARUCO_BOARD_3x4 selected
    {
      detected = calibration_board_detector_.detectArucoBoard(image_, 3, 4, cv::aruco::DICT_4X4_50,
                                                              marker_size_field_->text().toDouble(),
                                                              seperation_field_->text().toDouble());
      break;
    }
    case 6:  // CHARUCO_BOARD_5x7 selected
    {
      detected = calibration_board_detector_.detectCharucoBoard(image_, 5, 7, cv::aruco::DICT_6X6_250,
                                                                square_size_field_->text().toDouble(),
                                                                marker_size_field_->text().toDouble());
      break;
    }
    default:  // Other calibration board used
    {
      break;
    }
  }
  if (detected)
  {
    qimage = QImage(image_.data, image_.cols, image_.rows, QImage::Format_RGB888);
    tf2::Quaternion q;
    calibration_board_detector_.CVRvec_to_ROSTFquaternion(q);
    std::vector<double> t;
    calibration_board_detector_.get_tvect(t);
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = msg->header.frame_id;
    transformStamped.child_frame_id = "calib_board";
    transformStamped.transform.rotation.x = q.getX();
    transformStamped.transform.rotation.y = q.getY();
    transformStamped.transform.rotation.z = q.getZ();
    transformStamped.transform.rotation.w = q.getW();
    transformStamped.transform.translation.x = t[0];
    transformStamped.transform.translation.y = t[1];
    transformStamped.transform.translation.z = t[2];
    tfBroadcaster_.sendTransform(transformStamped);
  }
  else
    qimage = QImage(image_.data, image_.cols, image_.rows, QImage::Format_Grayscale8);

  image_view_->drawImage(qimage);
}

// *************************************************************************************************************
// Callback function for dealing with the camera_info message received from the topic camera_info_topic_field_
// *************************************************************************************************************
void HandeyeCalibrationWidget::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  calibration_board_detector_.setCameraMatrixFromMsg(msg);
}

// *******************************************************************************
// Received when the selected option in image_topic_field_ is changed
// *******************************************************************************
void HandeyeCalibrationWidget::imageTopicComboboxChanged(int index)
{
  image_sub_.shutdown();
  QString topic = image_topic_field_->currentText();
  if (!topic.isNull() and !topic.isEmpty())
  {
    try
    {
      image_sub_ = it_.subscribe(topic.toStdString(), 1, &HandeyeCalibrationWidget::imageCallback, this);
    }
    catch (image_transport::TransportLoadException& e)
    {
      ROS_ERROR_STREAM("Subscribe to image topic: " << topic.toStdString() << " failed. " << e.what());
    }
  }
}

// *******************************************************************************
// Received when the selected option in camera_info_topic_field_ is changed
// *******************************************************************************
void HandeyeCalibrationWidget::cameraInfoComboBoxChanged(int index)
{
  camerainfo_sub_.shutdown();
  QString topic = camera_info_topic_field_->currentText();
  if (!topic.isNull() and !topic.isEmpty())
  {
    try
    {
      camerainfo_sub_ = nh_.subscribe(topic.toStdString(), 1, &HandeyeCalibrationWidget::cameraInfoCallback, this);
    }
    catch (ros::Exception& e)
    {
      ROS_ERROR_STREAM("Subscribe to camera info topic: " << topic.toStdString() << " failed. " << e.what());
    }
  }
}

// *******************************************************************************
// Received when image_topic_field_ combobox is clicked
// *******************************************************************************
void ImageTopicComboBox::mousePressEvent(QMouseEvent* event)
{
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");

  // Get all topic names
  ros::master::V_TopicInfo ros_topic_vec;
  ros::master::getTopics(ros_topic_vec);

  // Filter out the image topic names
  QSet<QString> image_topics;
  for (auto it = ros_topic_vec.begin(); it != ros_topic_vec.end(); it++)
  {
    if (message_types.contains(QString(it->datatype.c_str())))
    {
      image_topics.insert(QString(it->name.c_str()));
    }
  }
  clear();
  addItem(QString(""));
  for (auto it = image_topics.begin(); it != image_topics.end(); it++)
  {
    addItem(*it);
  }
  showPopup();
}

// *******************************************************************************
// Received when camera_info_topic_field_ combobox is clicked
// *******************************************************************************
void CameraInfoTopicComboBox::mousePressEvent(QMouseEvent* event)
{
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/CameraInfo");

  // Get all topic names
  ros::master::V_TopicInfo ros_topic_vec;
  ros::master::getTopics(ros_topic_vec);

  // Filter out the image info topic names
  QSet<QString> camerainfo_topics;
  for (auto it = ros_topic_vec.begin(); it != ros_topic_vec.end(); it++)
  {
    if (message_types.contains(QString(it->datatype.c_str())))
    {
      camerainfo_topics.insert(QString(it->name.c_str()));
    }
  }
  clear();
  addItem(QString(""));
  for (auto it = camerainfo_topics.begin(); it != camerainfo_topics.end(); it++)
  {
    addItem(*it);
  }
  showPopup();
}

// *******************************************************************************
// Received when frame name combobox is clicked
// *******************************************************************************
void TFFrameNameComboBox::mousePressEvent(QMouseEvent* event)
{
  std::vector<std::string> names;

  tfBuffer_._getFrameStrings(names);

  clear();
  for (auto it = names.begin(); it != names.end(); it++)
  {
    addItem(QString((*it).c_str()));
  }
  showPopup();
}

// *******************************************************************************
// Received when image_view_ frame is updated
// *******************************************************************************
void ImageViewFrame::paintEvent(QPaintEvent* event)
{
  QPainter painter(this);
  image_mutex_.lock();
  if (!qimage_.isNull())
  {
    QRect rect = contentsRect();

    if (rect.width() / rect.height() > qimage_.width() / qimage_.height())
    {
      qimage_ = qimage_.scaledToHeight(rect.height(), Qt::SmoothTransformation);
    }
    else
    {
      qimage_ = qimage_.scaledToWidth(rect.width(), Qt::SmoothTransformation);
    }
    painter.drawImage(QPoint(0, 0), qimage_);
  }
  else
  {
    painter.setBrush(Qt::gray);
    painter.drawRect(contentsRect());
  }
  image_mutex_.unlock();
}

// *******************************************************************************
// Draw image in image_view_ frame
// *******************************************************************************
void ImageViewFrame::drawImage(const QImage& qimage)
{
  image_mutex_.lock();
  qimage_ = qimage.copy();
  image_mutex_.unlock();
  Q_EMIT update_view();
}

}  // namespace moveit_setup_assistant