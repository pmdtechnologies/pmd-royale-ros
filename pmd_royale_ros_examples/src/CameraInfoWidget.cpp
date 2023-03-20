/****************************************************************************\
 * Copyright (C) 2023 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#include "CameraInfoWidget.hpp"

#include <QHBoxLayout>
#include <QString>
#include <chrono>
#include <functional>
#include <regex>

using namespace std::chrono_literals;

namespace pmd_royale_ros_examples {

CameraInfoWidget::CameraInfoWidget(std::shared_ptr<rclcpp::Node> node, std::string cameraNodeName, QWidget *parent)
    : QWidget(parent), CameraParametersClient(node, cameraNodeName), m_nh(node) {
    QVBoxLayout *infoLayout = new QVBoxLayout(this);

    QHBoxLayout *serialLayout = new QHBoxLayout();
    serialLayout->addWidget(new QLabel("Serial :", this));
    m_labelCameraSerial = new QLabel("", this);
    serialLayout->addWidget(m_labelCameraSerial);
    infoLayout->addLayout(serialLayout);

    QHBoxLayout *modelLayout = new QHBoxLayout();
    modelLayout->addWidget(new QLabel("Model :", this));
    m_labelCameraModel = new QLabel("", this);
    modelLayout->addWidget(m_labelCameraModel);
    infoLayout->addLayout(modelLayout);

    subscribeForCameraParameters({"serial", "model"});
}

void CameraInfoWidget::onNewCameraParameter(const CameraParameter &cameraParam) {
    auto param = cameraParam.parameter;
    if (param->get_name() == "serial") {
        m_labelCameraSerial->setText(QString::fromStdString(param->as_string()));
    } else if (param->get_name() == "model") {
        m_labelCameraModel->setText(QString::fromStdString(param->as_string()));
    }
}

} // namespace pmd_royale_ros_examples
