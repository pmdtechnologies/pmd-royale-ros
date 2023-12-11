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

#define ROYALE_ROS_MAX_STREAMS 2u

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
    bool setExposureTime(int exposureTime, royale::StreamId streamId);
    bool enableAutoExposure(bool enable, royale::StreamId streamId);

    void initUseCase();

    void updateDataListeners();

    void setProcParams(const std_msgs::msg::String::SharedPtr parameters, uint32_t streamIdx);

    // Published topics
    sensor_msgs::msg::CameraInfo m_cameraInfo;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_pubCameraInfo;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pubCloud[ROYALE_ROS_MAX_STREAMS];
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pubDepth[ROYALE_ROS_MAX_STREAMS];
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pubGray[ROYALE_ROS_MAX_STREAMS];

    // Interface to configure actual camera
    std::unique_ptr<royale::ICameraDevice> m_cameraDevice;

    OnSetParametersCallbackHandle::SharedPtr m_onSetParametersCbHandle;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr m_onSetParametersEventCbHandle;
    rclcpp::SyncParametersClient m_parametersClient;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_procParamsSubscription[ROYALE_ROS_MAX_STREAMS];

    // Parameters
    std::string m_serial;
    std::string m_model;
    std::string m_currentUseCase;
    std::string m_startUseCase;
    int64_t m_exposureTime[ROYALE_ROS_MAX_STREAMS];
    bool m_isAutoExposureEnabled[ROYALE_ROS_MAX_STREAMS];
    bool m_isPubCloud;
    bool m_isPubDepth;
    bool m_isPubGray;
    bool m_registeredPCListener;
    bool m_registeredIRListener;
    rclcpp::TimerBase::SharedPtr m_updateDataListenersTimer;
    std::map<royale::StreamId, uint32_t> m_streamIdx;
    std::string m_recording_file;
};

} // namespace pmd_royale_ros_driver

#endif // __PMD_ROYALE_ROS_DRIVER__CAMERA_NODE_HPP__
