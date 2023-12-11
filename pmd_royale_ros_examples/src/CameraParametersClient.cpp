/****************************************************************************\
 * Copyright (C) 2023 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#include "CameraParametersClient.hpp"
#include <algorithm>
#include <functional>

namespace pmd_royale_ros_examples {

CameraParametersClient::CameraParametersClient(rclcpp::Node::SharedPtr node, std::string cameraNode)
    : m_nh(node), m_cameraNode(cameraNode) {
    m_parametersClient = std::make_shared<rclcpp::AsyncParametersClient>(m_nh, m_cameraNode);
    m_parameterEventSub = m_parametersClient->on_parameter_event(
        std::bind(&CameraParametersClient::onParameterEvent, this, std::placeholders::_1));
}

void CameraParametersClient::onParametersDescriptorResult(
    std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> future) {
    auto result = future.get();
    for (auto &descriptor : result) {
        if (m_subsribedParameters.find(descriptor.name) != m_subsribedParameters.end()) {
            auto &cameraParameter = m_parameters[descriptor.name];
            cameraParameter.descriptor = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>(descriptor);
            if (cameraParameter.parameter && cameraParameter.descriptor) {
                onNewCameraParameter(m_parameters[descriptor.name]);
            }
        }
    }
}

void CameraParametersClient::onParametersResult(std::shared_future<std::vector<rclcpp::Parameter>> future) {
    auto result = future.get();
    std::cout << "result from future: " << result << std::endl;
    for (auto &param : result) {
        std::cout << "param name: " << param.get_name();
        if (m_subsribedParameters.find(param.get_name()) != m_subsribedParameters.end() &&
            param.get_type() != rclcpp::PARAMETER_NOT_SET) {
            auto &cameraParameter = m_parameters[param.get_name()];
            cameraParameter.parameter =
                std::make_shared<rclcpp::Parameter>(param.get_name(), param.get_parameter_value());
            if (!cameraParameter.descriptor) {
                m_parametersClient->describe_parameters(
                    {param.get_name()},
                    std::bind(&CameraParametersClient::onParametersDescriptorResult, this, std::placeholders::_1));
            } else {
                onNewCameraParameter(m_parameters[param.get_name()]);
            }
        }
    }
}

void CameraParametersClient::onParameterEvent(const rcl_interfaces::msg::ParameterEvent &event) {
    for (auto &param : event.deleted_parameters) {
        if (m_parameters.find(param.name) != m_parameters.end()) {
            m_parameters.erase(param.name);
        }
    }

    for (auto &param : event.new_parameters) {
        if (m_parameters.find(param.name) != m_parameters.end()) {
            m_parameters.erase(param.name);
        }
    }

    auto params = rclcpp::ParameterEventHandler::get_parameters_from_event(event);
    for (auto &param : params) {
        if (m_subsribedParameters.find(param.get_name()) != m_subsribedParameters.end() &&
            param.get_type() != rclcpp::PARAMETER_NOT_SET) {
            auto &cameraParameter = m_parameters[param.get_name()];
            cameraParameter.parameter =
                std::make_shared<rclcpp::Parameter>(param.get_name(), param.get_parameter_value());

            if (!cameraParameter.descriptor) {
                m_parametersClient->describe_parameters(
                    {param.get_name()},
                    std::bind(&CameraParametersClient::onParametersDescriptorResult, this, std::placeholders::_1));
            } else {
                onNewCameraParameter(m_parameters[param.get_name()]);
            }
        }
    }
}

void CameraParametersClient::subscribeForCameraParameters(const std::set<std::string> &parameterNames) {
    std::cout << "Inside subscribe for camera Parameters, got these parameters: ";
    for (auto p : parameterNames){
        std::cout << p << ", ";
    }
    std::cout << " " << std::endl;
    std::set<std::string> newParameters;
    std::set_union(m_subsribedParameters.begin(), m_subsribedParameters.end(), parameterNames.begin(),
                   parameterNames.end(), std::inserter(newParameters, newParameters.begin()));
    m_subsribedParameters = newParameters;
    std::vector<std::string> parameters(m_subsribedParameters.begin(), m_subsribedParameters.end());
    std::cout << "Final parameters vector, got these parameters: ";
    for (auto p : parameters){
        std::cout << p << ", ";
    }
    std::cout << " " << std::endl;
    m_parametersClient->get_parameters(parameters, std::bind(&CameraParametersClient::onParametersResult, this, std::placeholders::_1));
}

void CameraParametersClient::setParameter(const rclcpp::Parameter &parameter) {
    m_parametersClient->set_parameters({parameter});
}
} // namespace pmd_royale_ros_examples
