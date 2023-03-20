/****************************************************************************\
 * Copyright (C) 2023 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/
#ifndef __PMD_ROYALE_ROS_DRIVER__CAMERA_NODE_HPP__
#define __PMD_ROYALE_ROS_DRIVER__CAMERA_NODE_HPP__

#include <royale.hpp>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include "VisibilityControl.hpp"

namespace pmd_royale_ros_driver {

class CameraNode : public rclcpp::Node,
                   public royale::IPointCloudListener,
                   public royale::IIRImageListener,
                   public royale::IExposureListener {
  public:
    PMD_ROYALE_ROS_DRIVER_PUBLIC
    CameraNode(const rclcpp::NodeOptions &options);
    ~CameraNode();

    // Starting and stopping the camera
    void start();
    void stop();

  private:

    // Callbacks from CameraDevice when image is ready
    void onNewData(const royale::PointCloud *data) override;
    void onNewData(const royale::IRImage *data) override;

    // Called by CameraDevice for every new exposure time when auto exposure is enabled.
    void onNewExposure(const uint32_t exposureTime, const royale::StreamId streamId) override;

    // Parameter set/events callbacks
    rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter> &parameters);
    void onParametersSetEvent(const rcl_interfaces::msg::ParameterEvent &event);

    // Create cameraInfo, return true if the setting is successful, otherwise false
    bool setCameraInfo();

    // Callbacks for parameter changes which reconfigure the CameraDevice
    bool setUseCase(const std::string &useCase);
    bool setExposureTime(int exposureTime);
    bool enableAutoExposure(bool enable);

    void updateDataListeners();

    // Published topics
    sensor_msgs::msg::CameraInfo m_cameraInfo;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_pubCameraInfo;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pubCloud;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pubDepth;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pubGray;

    // Interface to configure actual camera
    std::unique_ptr<royale::ICameraDevice> m_cameraDevice;

    OnSetParametersCallbackHandle::SharedPtr m_onSetParametersCbHandle;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr m_onSetParametersEventCbHandle;
    rclcpp::SyncParametersClient m_parametersClient;

    // Parameters
    std::string m_serial;
    std::string m_model;
    std::string m_currentUseCase;
    int64_t m_exposureTime;
    bool m_isAutoExposureEnabled;
    bool m_isPubCloud;
    bool m_isPubDepth;
    bool m_isPubGray;
    bool m_registeredPCListener;
    bool m_registeredIRListener;
    rclcpp::TimerBase::SharedPtr m_updateDataListenersTimer;
};

} // namespace pmd_royale_ros_driver

#endif // __PMD_ROYALE_ROS_DRIVER__CAMERA_NODE_HPP__
