/****************************************************************************\
 * Copyright (C) 2023 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#ifndef __PMD_ROYALE_ROS_EXAMPLES__CAMERA_PARAMETERS_CLIENT_HPP__
#define __PMD_ROYALE_ROS_EXAMPLES__CAMERA_PARAMETERS_CLIENT_HPP__

#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>
#include <unordered_map>

namespace pmd_royale_ros_examples {

/// Contains the data and descriptor which a GUI needs to represent one camera parameter of a camera node
struct CameraParameter {
    std::shared_ptr<rclcpp::Parameter> parameter;
    std::shared_ptr<rcl_interfaces::msg::ParameterDescriptor> descriptor;
};

/// Client to access a camera node's parameters and listen for updates.
class CameraParametersClient {
  public:
    CameraParametersClient(rclcpp::Node::SharedPtr node, std::string cameraNode);

    /// Subscribe for desired camera parameters by name
    void subscribeForCameraParameters(const std::set<std::string> &parameterNames);

    /// Submit change to camera parameter
    void setParameter(const rclcpp::Parameter &parameter);

  protected:
    void onParametersDescriptorResult(std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> future);
    void onParametersResult(std::shared_future<std::vector<rclcpp::Parameter>> future);
    void onParameterEvent(const rcl_interfaces::msg::ParameterEvent &event);

    /// Callback for when a new or updated CameraParameter is available
    virtual void onNewCameraParameter(const CameraParameter &parameter) = 0;

  private:
    rclcpp::Node::SharedPtr m_nh;
    std::string m_cameraNode;
    rclcpp::AsyncParametersClient::SharedPtr m_parametersClient;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr m_parameterEventSub;
    std::set<std::string> m_subsribedParameters;
    std::unordered_map<std::string, CameraParameter> m_parameters;
};

} // namespace pmd_royale_ros_examples

#endif // __PMD_ROYALE_ROS_EXAMPLES__CAMERA_PARAMETERS_CLIENT_HPP__
