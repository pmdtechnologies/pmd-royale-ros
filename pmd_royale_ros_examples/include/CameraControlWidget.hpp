/****************************************************************************\
 * Copyright (C) 2023 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#ifndef __PMD_ROYALE_ROS_EXAMPLES__CAMERA_CONTROL_WIDGET_HPP__
#define __PMD_ROYALE_ROS_EXAMPLES__CAMERA_CONTROL_WIDGET_HPP__

#include "CameraParametersClient.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QSlider>
#include <QWidget>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#define ROYALE_ROS_MAX_STREAMS 2u

namespace pmd_royale_ros_examples {

class CameraControlWidget : public QWidget, public CameraParametersClient {
    Q_OBJECT
  public:
    CameraControlWidget(std::shared_ptr<rclcpp::Node> node, std::string cameraNodeName, QWidget *parent = 0);
    ~CameraControlWidget();

  private Q_SLOTS:
    void setUseCase(const QString &currentMode);
    void setExposureTime(int value, uint32_t streamIdx);
    void setExposureMode(bool isAutomatic, uint32_t streamIdx);

    // The precise value can be entered directly via the text editor.
    void preciseExposureTimeSetting(uint32_t streamIdx);

  private:
    virtual void onNewCameraParameter(const CameraParameter &cameraParam) override;

    QComboBox *m_comboBoxUseCases;
    QLabel *m_labelExpoTime[ROYALE_ROS_MAX_STREAMS];
    QSlider *m_sliderExpoTime[ROYALE_ROS_MAX_STREAMS];
    QLineEdit *m_lineEditExpoTime[ROYALE_ROS_MAX_STREAMS];
    QCheckBox *m_checkBoxAutoExpo[ROYALE_ROS_MAX_STREAMS];
    rclcpp::Node::SharedPtr m_nh;
};

} // namespace pmd_royale_ros_examples

#endif // __PMD_ROYALE_ROS_EXAMPLES__CAMERA_CONTROL_WIDGET_HPP__
