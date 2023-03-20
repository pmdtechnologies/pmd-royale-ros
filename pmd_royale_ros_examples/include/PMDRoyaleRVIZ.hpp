/****************************************************************************\
 * Copyright (C) 2023 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#ifndef __PMD_ROYALE_ROS_EXAMPLES__PMD_ROYALE_RVIZ_HPP__
#define __PMD_ROYALE_ROS_EXAMPLES__PMD_ROYALE_RVIZ_HPP__

#include "CameraControlWidget.hpp"
#include "CameraInfoWidget.hpp"

#include <thread>

#include <QLabel>
#include <QLineEdit>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace pmd_royale_ros_examples {

/// RVIZ panel for configuring a camera node
class PMDRoyaleRVIZ : public rviz_common::Panel {
    Q_OBJECT
  public:
    PMDRoyaleRVIZ(QWidget *parent = 0);
    ~PMDRoyaleRVIZ();

  private Q_SLOTS:
    void handleCameraNodeSetting();
    virtual void load(const rviz_common::Config &config) override;
    virtual void save(rviz_common::Config config) const override;

  private:
    void init();
    void spin();

    QString m_cameraNode;
    QLabel *m_labelCameraNode;
    QLineEdit *m_lineEditCameraNode;
    CameraControlWidget *m_cameraControlWidget;
    CameraInfoWidget *m_cameraInfoWidget;
    QTabWidget *m_tabWidget;
    rclcpp::Node::SharedPtr m_nh;
    rclcpp::executors::SingleThreadedExecutor m_exec;
    std::thread m_thread;
};

} // namespace pmd_royale_ros_examples

#endif // __PMD_ROYALE_ROS_EXAMPLES__PMD_ROYALE_RVIZ_HPP__
