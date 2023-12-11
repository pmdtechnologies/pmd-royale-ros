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
#include <QScrollArea>
#include <QString>
#include <functional>
#include <QVBoxLayout>

using namespace std::chrono_literals;

namespace pmd_royale_ros_examples {

PMDRoyaleRVIZ::PMDRoyaleRVIZ(QWidget *parent)
    : rviz_common::Panel(parent),
      m_nh(rclcpp::Node::make_shared("PMDRoyaleRVIZ")) {

    m_cameraNode = QString::fromStdString("");
    std::set<std::string> cameras;

    auto node_names = m_nh->get_node_names();
    for (const auto& node : node_names){
        if(node.find("pmd_camera_") != std::string::npos){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Available Camera : %s", node.c_str());
            m_cameraNode = QString::fromStdString(node);
            cameras.emplace(node);
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Selected camera : %s", m_cameraNode.toStdString().c_str());
    
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->setSizeConstraint(QLayout::SetMinimumSize);

    // Top section to specify camera node name
    QVBoxLayout *topLayout = new QVBoxLayout();
    mainLayout->addLayout(topLayout);

    // Bottom section for camera info and controls
    QVBoxLayout *bottomLayout = new QVBoxLayout();
    m_tabWidget = new QTabWidget(this);
    bottomLayout->addWidget(m_tabWidget);
    mainLayout->addLayout(bottomLayout);

    // Camera
    topLayout->addWidget(new QLabel("Camera:"));
    m_cams = new QComboBox;

    for (auto curCam : cameras) {
        m_cams->addItem(curCam.c_str());
    }
    topLayout->addWidget(m_cams);

    connect(m_cams, SIGNAL(currentIndexChanged(int)), this, SLOT(chooseCamera(int)));
    m_cams->currentIndexChanged(0);

    init();

    m_thread = std::thread(&PMDRoyaleRVIZ::spin, this);
}

PMDRoyaleRVIZ::~PMDRoyaleRVIZ() {
    m_exec.cancel();
    m_thread.join();
}

void PMDRoyaleRVIZ::chooseCamera(int idx) {
    auto cameraName = m_cams->itemText(idx).toStdString();
    if (cameraName != m_cameraNode.toStdString()) {
        m_cameraNode = QString::fromStdString(cameraName);
        std::cout << "Chose Camera: " << cameraName << std::endl;
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
        m_cams->setCurrentIndex(m_cams->findText(m_cameraNode));
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
