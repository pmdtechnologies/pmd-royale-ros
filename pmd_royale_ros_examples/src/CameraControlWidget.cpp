/****************************************************************************\
 * Copyright (C) 2023 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#include "CameraControlWidget.hpp"

#include <QHBoxLayout>

namespace pmd_royale_ros_examples {

CameraControlWidget::CameraControlWidget(std::shared_ptr<rclcpp::Node> node, std::string cameraNodeName,
                                         QWidget *parent)
    : QWidget(parent), CameraParametersClient(node, cameraNodeName), m_nh(node) {
    QVBoxLayout *controlLayout = new QVBoxLayout(this);
    // Use Case
    controlLayout->addWidget(new QLabel("Use Case:"));
    m_comboBoxUseCases = new QComboBox;
    controlLayout->addWidget(m_comboBoxUseCases);

    // Exposure Time
    m_labelExpoTime = new QLabel;
    controlLayout->addWidget(m_labelExpoTime);
    QHBoxLayout *exTimeLayout = new QHBoxLayout;

    m_sliderExpoTime = new QSlider(Qt::Horizontal);
    m_sliderExpoTime->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
    exTimeLayout->addWidget(m_sliderExpoTime);

    m_lineEditExpoTime = new QLineEdit;
    m_lineEditExpoTime->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
    exTimeLayout->addWidget(m_lineEditExpoTime);

    controlLayout->addLayout(exTimeLayout);

    // Auto Exposure
    controlLayout->addWidget(new QLabel("Auto Exposure:"));
    m_checkBoxAutoExpo = new QCheckBox;
    controlLayout->addWidget(m_checkBoxAutoExpo);

    setLayout(controlLayout);

    connect(m_comboBoxUseCases, SIGNAL(currentTextChanged(const QString)), this, SLOT(setUseCase(const QString)));
    connect(m_comboBoxUseCases, SIGNAL(currentTextChanged(const QString)), this, SLOT(setUseCase(const QString)));
    connect(m_sliderExpoTime, SIGNAL(valueChanged(int)), this, SLOT(setExposureTime(int)));
    connect(m_checkBoxAutoExpo, SIGNAL(toggled(bool)), this, SLOT(setExposureMode(bool)));
    connect(m_lineEditExpoTime, SIGNAL(editingFinished()), this, SLOT(preciseExposureTimeSetting()));

    subscribeForCameraParameters({"available_usecases", "usecase", "exposure_time", "auto_exposure",
                                  "gray_image_divisor", "min_distance_filter", "max_distance_filter"});
}

CameraControlWidget::~CameraControlWidget() {}

void CameraControlWidget::onNewCameraParameter(const CameraParameter &cameraParam) {
    auto param = cameraParam.parameter;
    auto descriptor = cameraParam.descriptor;

    if (param->get_name() == "available_usecases") {
        std::vector<std::string> availableUseCases = param->as_string_array();
        m_comboBoxUseCases->blockSignals(true);
        m_comboBoxUseCases->clear();
        for (auto useCase : availableUseCases) {
            m_comboBoxUseCases->addItem(QString::fromStdString(useCase));
        }
        m_comboBoxUseCases->blockSignals(false);
    } else if (param->get_name() == "usecase") {
        RCLCPP_INFO(m_nh->get_logger(), "Current usecase: %s", param->as_string().c_str());
        m_comboBoxUseCases->blockSignals(true);
        int currentIndex = m_comboBoxUseCases->findText(param->as_string().c_str());
        if (currentIndex != -1) {
            m_comboBoxUseCases->setCurrentIndex(currentIndex);
        }
        m_comboBoxUseCases->blockSignals(false);
    } else if (param->get_name() == "exposure_time") {
        auto exposureRange = descriptor->integer_range.front();
        m_sliderExpoTime->blockSignals(true);
        m_sliderExpoTime->setRange(exposureRange.from_value, exposureRange.to_value);
        m_labelExpoTime->setText("Exposure Time (microseconds):");
        m_sliderExpoTime->setValue(param->as_int());
        m_sliderExpoTime->blockSignals(false);

        m_lineEditExpoTime->blockSignals(true);
        m_lineEditExpoTime->setText(QString::number(param->as_int()));
        m_lineEditExpoTime->blockSignals(false);
    }
}

void CameraControlWidget::setUseCase(const QString &currentMode) {
    rclcpp::Parameter parameter("usecase", currentMode.toStdString());
    setParameter(parameter);
}

void CameraControlWidget::setExposureTime(int value) {
    rclcpp::Parameter parameter("exposure_time", value);
    setParameter(parameter);
}

void CameraControlWidget::setExposureMode(bool isAutomatic) {
    rclcpp::Parameter parameter("auto_exposure", isAutomatic);
    setParameter(parameter);
}

void CameraControlWidget::preciseExposureTimeSetting() {
    int value = m_lineEditExpoTime->text().toInt();
    setExposureTime(value);
}

} // namespace pmd_royale_ros_examples
