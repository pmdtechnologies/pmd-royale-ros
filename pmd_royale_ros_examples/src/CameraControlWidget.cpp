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

    connect(m_comboBoxUseCases, SIGNAL(currentTextChanged(const QString)), this, SLOT(setUseCase(const QString)));
    connect(m_comboBoxUseCases, SIGNAL(currentTextChanged(const QString)), this, SLOT(setUseCase(const QString)));

    for (auto i = 0u; i < ROYALE_ROS_MAX_STREAMS; ++i) {
        controlLayout->addWidget(new QLabel(QString("Stream ") + QString::number(i) + " : "));

        // Exposure Time
        m_labelExpoTime[i] = new QLabel;
        controlLayout->addWidget(m_labelExpoTime[i]);
        QHBoxLayout *exTimeLayout = new QHBoxLayout;

        m_sliderExpoTime[i] = new QSlider(Qt::Horizontal);
        m_sliderExpoTime[i]->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        m_sliderExpoTime[i]->setTracking(false);
        exTimeLayout->addWidget(m_sliderExpoTime[i]);

        m_lineEditExpoTime[i] = new QLineEdit;
        m_lineEditExpoTime[i]->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
        exTimeLayout->addWidget(m_lineEditExpoTime[i]);

        controlLayout->addLayout(exTimeLayout);

        // Auto Exposure
        controlLayout->addWidget(new QLabel("Auto Exposure:"));
        m_checkBoxAutoExpo[i] = new QCheckBox;
        controlLayout->addWidget(m_checkBoxAutoExpo[i]);

        setLayout(controlLayout);

        connect(m_sliderExpoTime[i], &QSlider::valueChanged, this, [this, i](int val) { setExposureTime(val, i); });
        connect(m_checkBoxAutoExpo[i], &QCheckBox::toggled, this, [this, i](bool val) { setExposureMode(val, i); });
        connect(m_lineEditExpoTime[i], &QLineEdit::editingFinished, this, [this, i](void) { preciseExposureTimeSetting(i); });
    }
    subscribeForCameraParameters({"available_usecases", "usecase",
                                  "gray_image_divisor", "min_distance_filter", "max_distance_filter"});
    for (auto i = 0u; i < ROYALE_ROS_MAX_STREAMS; ++i) {
        subscribeForCameraParameters({"exposure_time_" + std::to_string(i), "auto_exposure_" + std::to_string(i)});
    }
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
    } else if (param->get_name().find("exposure_time_") == 0) {
        auto streamIdxStr = param->get_name().substr(strlen("exposure_time_"));
        auto streamIdx = stoi(streamIdxStr);
        auto exposureRange = descriptor->integer_range.front();
        m_sliderExpoTime[streamIdx]->blockSignals(true);
        m_sliderExpoTime[streamIdx]->setRange(exposureRange.from_value, exposureRange.to_value);
        m_labelExpoTime[streamIdx]->setText("Exposure Time (microseconds):");
        m_sliderExpoTime[streamIdx]->setValue(param->as_int());
        m_sliderExpoTime[streamIdx]->blockSignals(false);

        m_lineEditExpoTime[streamIdx]->blockSignals(true);
        m_lineEditExpoTime[streamIdx]->setText(QString::number(param->as_int()));
        m_lineEditExpoTime[streamIdx]->blockSignals(false);
    } else if (param->get_name().find("auto_exposure_") == 0) {
        auto streamIdxStr = param->get_name().substr(strlen("auto_exposure_"));
        auto streamIdx = stoi(streamIdxStr);
        m_checkBoxAutoExpo[streamIdx]->blockSignals(true);
        m_checkBoxAutoExpo[streamIdx]->setChecked(param->as_bool());
        m_sliderExpoTime[streamIdx]->setEnabled(!param->as_bool());
        m_labelExpoTime[streamIdx]->setEnabled(!param->as_bool());
        m_checkBoxAutoExpo[streamIdx]->blockSignals(false);
    }
}

void CameraControlWidget::setUseCase(const QString &currentMode) {
    rclcpp::Parameter parameter("usecase", currentMode.toStdString());
    setParameter(parameter);
}

void CameraControlWidget::setExposureTime(int value, uint32_t streamIdx) {
    rclcpp::Parameter parameter(std::string("exposure_time_") + std::to_string(streamIdx), value);
    setParameter(parameter);
}

void CameraControlWidget::setExposureMode(bool isAutomatic, uint32_t streamIdx) {
    rclcpp::Parameter parameter(std::string("auto_exposure_") + std::to_string(streamIdx), isAutomatic);
    setParameter(parameter);
}

void CameraControlWidget::preciseExposureTimeSetting(uint32_t streamIdx) {
    int value = m_lineEditExpoTime[streamIdx]->text().toInt();
    setExposureTime(value, streamIdx);
}

} // namespace pmd_royale_ros_examples
