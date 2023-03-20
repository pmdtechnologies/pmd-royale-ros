/****************************************************************************\
 * Copyright (C) 2023 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#ifndef __PMD_ROYALE_ROS_EXAMPLES__CAMERA_INFO_WIDGET_HPP__
#define __PMD_ROYALE_ROS_EXAMPLES__CAMERA_INFO_WIDGET_HPP__

#include "CameraParametersClient.hpp"

#include <QLabel>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace pmd_royale_ros_examples {

class CameraInfoWidget : public QWidget, public CameraParametersClient {
    Q_OBJECT
  public:
    CameraInfoWidget(std::shared_ptr<rclcpp::Node> node, std::string cameraNodeName, QWidget *parent = 0);

  private:
    virtual void onNewCameraParameter(const CameraParameter &param) override;

    QLabel *m_labelCameraSerial;
    QLabel *m_labelCameraModel;

    rclcpp::Node::SharedPtr m_nh;
};

} // namespace pmd_royale_ros_examples

#endif // __PMD_ROYALE_ROS_EXAMPLES__CAMERA_INFO_WIDGET_HPP__
