/****************************************************************************\
 * Copyright (C) 2023 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#include "PMDRoyaleRVIZ.hpp"

#include <QHBoxLayout>
#include <QString>
#include <functional>

using namespace std::chrono_literals;

namespace pmd_royale_ros_examples {

PMDRoyaleRVIZ::PMDRoyaleRVIZ(QWidget *parent)
    : rviz_common::Panel(parent),
      m_cameraNode("/pmd_royale_ros_camera_node"),
      m_nh(rclcpp::Node::make_shared("PMDRoyaleRVIZ")) {
    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    // Top section to specify camera node name
    QVBoxLayout *topLayout = new QVBoxLayout();
    QHBoxLayout *cameraNodeLayout = new QHBoxLayout();
    m_labelCameraNode = new QLabel("Camera Node: ", this);
    m_lineEditCameraNode = new QLineEdit(m_cameraNode, this);
    cameraNodeLayout->addWidget(m_labelCameraNode);
    cameraNodeLayout->addWidget(m_lineEditCameraNode);
    connect(m_lineEditCameraNode, SIGNAL(editingFinished()), this, SLOT(handleCameraNodeSetting()));
    topLayout->addLayout(cameraNodeLayout);
    mainLayout->addLayout(topLayout);

    // Bottom section for camera info and controls
    QVBoxLayout *bottomLayout = new QVBoxLayout();
    m_tabWidget = new QTabWidget(this);
    bottomLayout->addWidget(m_tabWidget);
    mainLayout->addLayout(bottomLayout);

    init();

    m_thread = std::thread(&PMDRoyaleRVIZ::spin, this);
}

PMDRoyaleRVIZ::~PMDRoyaleRVIZ() {
    m_exec.cancel();
    m_thread.join();
}

void PMDRoyaleRVIZ::handleCameraNodeSetting() {
    auto cameraName = m_lineEditCameraNode->text().toStdString();
    if (cameraName != m_cameraNode.toStdString()) {
        m_cameraNode = QString::fromStdString(cameraName);
        init();
    }
}

void PMDRoyaleRVIZ::init() {
    // Clear any existing widgets for existing camera node
    for (int i = m_tabWidget->count(); i > 0; i--) {
        m_tabWidget->widget(i - 1)->deleteLater();
        m_tabWidget->removeTab(i - 1);
    }

    m_cameraControlWidget = new CameraControlWidget(m_nh, m_cameraNode.toStdString(), m_tabWidget);
    m_tabWidget->addTab(m_cameraControlWidget, "Control");

    m_cameraInfoWidget = new CameraInfoWidget(m_nh, m_cameraNode.toStdString(), m_tabWidget);
    m_tabWidget->addTab(m_cameraInfoWidget, "Info");
}

void PMDRoyaleRVIZ::spin() {
    m_exec.add_node(m_nh);
    m_exec.spin();
    m_exec.remove_node(m_nh);
}

void PMDRoyaleRVIZ::load(const rviz_common::Config &config) {
    rviz_common::Panel::load(config);
    if (config.mapGetString("camera_node", &m_cameraNode)) {
        m_lineEditCameraNode->setText(m_cameraNode);
        init();
    }
}

void PMDRoyaleRVIZ::save(rviz_common::Config config) const {
    rviz_common::Panel::save(config);
    config.mapSetValue("camera_node", m_cameraNode);
}

} // namespace pmd_royale_ros_examples

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(pmd_royale_ros_examples::PMDRoyaleRVIZ, rviz_common::Panel)
