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
#include <std_msgs/msg/string.hpp>

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
    void setProcParameter(uint32_t streamIdx);

    // The precise value can be entered directly via the text editor.
    void preciseExposureTimeSetting(uint32_t streamIdx);

  private:
    virtual void onNewCameraParameter(const CameraParameter &cameraParam) override;
    
    void callbackInit(const std_msgs::msg::String::SharedPtr msg);
    
    // Callback the current parameters of exposure time
    // when the UI is started or user case is switched
    void callbackExpoTimeParam(const std_msgs::msg::String::SharedPtr msg, uint32_t streamIdx);
    void callbackExpoTimeValue(const std_msgs::msg::String::SharedPtr msg, uint32_t streamIdx);

    // Callback the fps and display it
    void callbackFps(const std_msgs::msg::String::SharedPtr msg, uint32_t streamIdx);

    QComboBox *m_comboBoxUseCases;
    QLabel *m_labelExpoTime[ROYALE_ROS_MAX_STREAMS];
    QSlider *m_sliderExpoTime[ROYALE_ROS_MAX_STREAMS];
    QLineEdit *m_lineEditExpoTime[ROYALE_ROS_MAX_STREAMS];
    QCheckBox *m_checkBoxAutoExpo[ROYALE_ROS_MAX_STREAMS];
    QLineEdit *m_lineEditParams[ROYALE_ROS_MAX_STREAMS];
    QLabel *m_labelEditFps[ROYALE_ROS_MAX_STREAMS];
    QSlider *m_sliderDivisor;
    QLineEdit *m_lineEditDivisor;
    QSlider *m_sliderMinFilter;
    QLineEdit *m_lineEditMinFilter;
    QSlider *m_sliderMaxFilter;
    QLineEdit *m_lineEditMaxFilter;
    QString m_currentUseCase;


    rclcpp::Node::SharedPtr m_nh;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubParameters[ROYALE_ROS_MAX_STREAMS];
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubIsInit;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubUseCase;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubExpoTime[ROYALE_ROS_MAX_STREAMS];
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubExpoMode[ROYALE_ROS_MAX_STREAMS];
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubMinFilter;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubMaxFilter;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubDivisor;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubParams[ROYALE_ROS_MAX_STREAMS];

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subInit;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subTemp;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subExpoTimeParam[ROYALE_ROS_MAX_STREAMS];
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subExpoTimeValue[ROYALE_ROS_MAX_STREAMS];
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subFps[ROYALE_ROS_MAX_STREAMS];

    bool m_isInit;
    bool m_isAutomatic[ROYALE_ROS_MAX_STREAMS];
    int m_exposureTime[ROYALE_ROS_MAX_STREAMS];
    int m_minETSlider[ROYALE_ROS_MAX_STREAMS];
    int m_maxETSlider[ROYALE_ROS_MAX_STREAMS];
};

} // namespace pmd_royale_ros_examples

#endif // __PMD_ROYALE_ROS_EXAMPLES__CAMERA_CONTROL_WIDGET_HPP__
