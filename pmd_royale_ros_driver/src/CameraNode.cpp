/****************************************************************************\
 * Copyright (C) 2023 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#include <CameraNode.hpp>
#include <limits.h>
#include <sstream>

using namespace std;
using namespace royale;

namespace pmd_royale_ros_driver {

CameraNode::CameraNode(const rclcpp::NodeOptions &options)
    : Node("pmd_royale_ros_camera_node", options),
      IExposureListener(),
      m_parametersClient(this),
      m_isPubCloud(false),
      m_isPubDepth(false),
      m_isPubGray(false),
      m_registeredPCListener(false),
      m_registeredIRListener(false),
      m_startUseCase(""),
      m_recording_file("") {

    unsigned int major;
    unsigned int minor;
    unsigned int patch;
    unsigned int build;
    royale::getVersion(major, minor, patch, build);
    RCLCPP_INFO(this->get_logger(), "Using Royale version %d.%d.%d.%d", major, minor, patch, build);

    rcl_interfaces::msg::ParameterDescriptor serialParameterDescriptor;
    serialParameterDescriptor.name = "serial";
    serialParameterDescriptor.description = "Serial number for the camera. Cannot change value after node configuration.";
    this->declare_parameter("serial", "", serialParameterDescriptor);

    rcl_interfaces::msg::ParameterDescriptor accessCodeParameterDescriptor;
    accessCodeParameterDescriptor.name = "access_code";
    accessCodeParameterDescriptor.description = "Access code for Royale.";
    this->declare_parameter("access_code", "", accessCodeParameterDescriptor);

    auto accessCode = this->get_parameter("access_code").as_string();

    if (!accessCode.empty()) {
        RCLCPP_INFO(this->get_logger(), "Using access code : %s", accessCode.c_str());
    }

    rcl_interfaces::msg::ParameterDescriptor startUseCaseParameterDescriptor;
    startUseCaseParameterDescriptor.name = "startUseCase";
    startUseCaseParameterDescriptor.description = "Start Usecase.";
    this->declare_parameter("startUseCase", "", startUseCaseParameterDescriptor);

    auto startUsecase = this->get_parameter("startUseCase").as_string();

    if(!startUsecase.empty()){
        RCLCPP_INFO(this->get_logger(), "Using Usecase: %s", startUsecase.c_str());
    }    

    rcl_interfaces::msg::ParameterDescriptor recFileParameterDescriptor;
    recFileParameterDescriptor.name = "recording_file";
    recFileParameterDescriptor.description = "Recording file that should be used.";
    this->declare_parameter("recording_file", "", recFileParameterDescriptor);

    if (!this->get_parameter("recording_file").as_string().empty()) {
        m_recording_file = this->get_parameter("recording_file").as_string();
    }

    CameraManager manager(accessCode.c_str());
    Vector<String> cameraList(manager.getConnectedCameraList());
    if (cameraList.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No suitable cameras found!");
        return;
    }

    // If serial is empty/not set, then pick the first camera that CameraManager probes
    if (this->get_parameter("serial").as_string().empty()) {
        this->set_parameter(rclcpp::Parameter("serial", cameraList[0].toStdString()));
    }
    m_serial = this->get_parameter("serial").as_string();

    int numCamsConnected = cameraList.size(); 
    RCLCPP_INFO(this->get_logger(), "%d cameras found!", numCamsConnected);

    m_cameraDevice = manager.createCamera(m_serial);
    if (m_cameraDevice) {
        RCLCPP_INFO(this->get_logger(), "Connected camera serial : %s", m_serial.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Could not connect to camera with serial %s", m_serial.c_str());
        return;
    }

    if (m_cameraDevice->initialize(m_startUseCase) != CameraStatus::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Error initializing the camera!");
        return;
    }

    royale::String cameraName;
    if (m_cameraDevice->getCameraName(cameraName) != royale::CameraStatus::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Could not get camera name for camera with serial %s", m_serial.c_str());
        return;
    }
    rcl_interfaces::msg::ParameterDescriptor modelParameterDescriptor;
    modelParameterDescriptor.name = "model";
    modelParameterDescriptor.description = "Model for the camera.";
    modelParameterDescriptor.read_only = true;
    m_model = this->declare_parameter("model", cameraName.toStdString(), modelParameterDescriptor, true);

    // Get the list of available usecases(modes). Filter out multi-stream usecases until it can be properly supported.
    Vector<String> useCases;
    if (m_cameraDevice->getUseCases(useCases) != royale::CameraStatus::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Could not get available usecases");
        return;
    }

    std::vector<std::string> stdUseCaseList;
    for (auto &useCase : useCases) {
        stdUseCaseList.push_back(useCase.toStdString());
    }

    rcl_interfaces::msg::ParameterDescriptor availableUseCasesParameterDescriptor;
    availableUseCasesParameterDescriptor.name = "available_usecases";
    availableUseCasesParameterDescriptor.description = "Read only list of available usecases for this camera";
    availableUseCasesParameterDescriptor.read_only = true;
    this->declare_parameter("available_usecases", stdUseCaseList, availableUseCasesParameterDescriptor, true);

    rcl_interfaces::msg::ParameterDescriptor currentUseCaseParameterDescriptor;
    currentUseCaseParameterDescriptor.name = "usecase";
    currentUseCaseParameterDescriptor.description = "Current usecase";
    this->declare_parameter("usecase", "", currentUseCaseParameterDescriptor);

    auto currentUseCaseParam = this->get_parameter("usecase");
    if (currentUseCaseParam.as_string().empty()) {
        String currentUseCase;
        if (m_cameraDevice->getCurrentUseCase(currentUseCase) != royale::CameraStatus::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Could not get current usecase");
            return;
        }
        this->set_parameter(rclcpp::Parameter("usecase", currentUseCase.toStdString()));
    } else {
        if (m_cameraDevice->setUseCase(royale::String::fromStdString(currentUseCaseParam.as_string())) != royale::CameraStatus::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Could not set usecase %s", currentUseCaseParam.as_string().c_str());
            return;
        }
    }

    Vector<StreamId> streamIds;
    if (m_cameraDevice->getStreams(streamIds) != CameraStatus::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't retrieve streams!");
        return;
    }
    for (auto i = 0u; i < streamIds.size(); ++i) {
        m_streamIdx[streamIds[i]] = i;
    }

    for (auto i = 0u; i < ROYALE_ROS_MAX_STREAMS; ++i) {
        rcl_interfaces::msg::ParameterDescriptor enableAEParamDescriptor;
        enableAEParamDescriptor.name = "auto_exposure_" + std::to_string(i);
        enableAEParamDescriptor.description = "Controls auto exposure for stream " + std::to_string(i);
        enableAEParamDescriptor.additional_constraints = "Cannot set the exposure_time parameter while this paramter's value is True";
        m_isAutoExposureEnabled[i] = this->declare_parameter("auto_exposure_" + std::to_string(i), true, enableAEParamDescriptor);
        royale::ExposureMode expoMode = m_isAutoExposureEnabled[i] ? royale::ExposureMode::AUTOMATIC : royale::ExposureMode::MANUAL;
        if (i < streamIds.size() && m_cameraDevice->setExposureMode(expoMode, streamIds[i]) != royale::CameraStatus::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Could not configure exposure mode for stream %d", i);
            return;
        }

        // If user provided exposure_time and no auto_exposure,
        royale::Pair<uint32_t, uint32_t> exposureLimits;
        m_cameraDevice->getExposureLimits(exposureLimits, streamIds[i]);
        rcl_interfaces::msg::ParameterDescriptor exposureParamDescriptor;
        exposureParamDescriptor.name = "exposure_time_" + std::to_string(i);
        exposureParamDescriptor.description = "Current exposure time for stream %d", i;
        exposureParamDescriptor.additional_constraints = "Cannot be set if auto_exposure is True. "
                                                         "Must be within the integer range for the current usecase.";
        rcl_interfaces::msg::IntegerRange exposureTimeRange;
        exposureTimeRange.from_value = exposureLimits.first;
        exposureTimeRange.to_value = exposureLimits.second;
        exposureTimeRange.step = 1;
        exposureParamDescriptor.integer_range.push_back(exposureTimeRange);
        exposureParamDescriptor.dynamic_typing = true; // Set dynamic_typing to true only so this can be re-declared later
        m_exposureTime[i] = this->declare_parameter(exposureParamDescriptor.name, (int)exposureLimits.second, exposureParamDescriptor, m_isAutoExposureEnabled[i]);
        if (i < streamIds.size() && !m_isAutoExposureEnabled[i]) {
            if (m_cameraDevice->setExposureTime((uint32_t)m_exposureTime[i], streamIds[i]) != royale::CameraStatus::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Could not set exposure time of %d for stream %d", (int)m_exposureTime[i], i);
                return;
            }
        }
    }

    if (!setCameraInfo()) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't create camera info!");
        return;
    }

    if (m_cameraDevice->registerExposureListener(this) != CameraStatus::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't register exposure listener!");
        return;
    }

    m_updateDataListenersTimer = this->create_wall_timer(std::chrono::milliseconds(250),
                                                         std::bind(&CameraNode::updateDataListeners, this));
    m_updateDataListenersTimer->call();

    std::string nodeName = this->get_name();
    std::replace(nodeName.begin(), nodeName.end(), '-', '_');
    RCLCPP_INFO(this->get_logger(), "Nodename : %s", nodeName.c_str());

    // Advertise our point cloud topic and image topics
    m_pubCameraInfo = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        nodeName + "/camera_info", 10);
    for (auto i = 0u; i < ROYALE_ROS_MAX_STREAMS; ++i) {
        m_pubCloud[i] = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            nodeName + "/point_cloud_" + std::to_string(i), 10);
        m_pubDepth[i] = this->create_publisher<sensor_msgs::msg::Image>(nodeName + "/depth_image_" + std::to_string(i),
                                                                        10);
        m_pubGray[i] = this->create_publisher<sensor_msgs::msg::Image>(nodeName + "/gray_image_" + std::to_string(i),
                                                                       10);

        std::function<void(const std_msgs::msg::String::SharedPtr msg)> fcn = std::bind(&CameraNode::setProcParams, this, std::placeholders::_1, i);
        m_procParamsSubscription[i] = this->create_subscription<std_msgs::msg::String>(
            nodeName + "/proc_params_" + std::to_string(i), 10, fcn);
    }

    m_onSetParametersCbHandle = this->add_on_set_parameters_callback(std::bind(&CameraNode::onSetParameters, this, std::placeholders::_1));
    m_onSetParametersEventCbHandle = m_parametersClient.on_parameter_event(std::bind(&CameraNode::onParametersSetEvent, this, std::placeholders::_1));

    initUseCase();

    start();
}

CameraNode::~CameraNode() {
    stop();
}

void CameraNode::start() {
    if (m_cameraDevice->startCapture() != CameraStatus::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Error starting camera capture!");
        return;
    }

    // If we specified a file start the recording
    if (!(m_recording_file.empty())) {
        char resolvedPath[PATH_MAX];
        realpath(m_recording_file.c_str(), resolvedPath);
        RCLCPP_INFO(this->get_logger(), "Recording to : %s", resolvedPath);
        m_cameraDevice->startRecording(m_recording_file);
    }
}

void CameraNode::stop() {
    // Close the camera
    if (m_cameraDevice && m_cameraDevice->stopCapture() != CameraStatus::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Error stopping camera capture!");
        return;
    }
}

void CameraNode::onNewData(const royale::PointCloud *data) {
    auto curIdx = m_streamIdx[data->streamId];

    std_msgs::msg::Header header;
    header.frame_id = "pmd_royale_ros_camera_node_optical_frame";
    header.stamp = rclcpp::Time(
        (chrono::duration_cast<chrono::nanoseconds>(chrono::microseconds(data->timestamp))).count());

    auto numPoints = data->getNumPoints();
    if (m_isPubCloud) {
        sensor_msgs::msg::PointCloud2::UniquePtr msgPointCloud(new sensor_msgs::msg::PointCloud2);
        msgPointCloud->header = header;
        msgPointCloud->width = data->height;
        msgPointCloud->height = data->width;
        msgPointCloud->is_bigendian = false;
        msgPointCloud->is_dense = false;
        msgPointCloud->point_step = static_cast<uint32_t>(4 * sizeof(float));
        msgPointCloud->row_step = static_cast<uint32_t>(4 * sizeof(float) * data->width);
        msgPointCloud->data.resize(4 * sizeof(float) * numPoints);

        sensor_msgs::PointCloud2Modifier modifier(*msgPointCloud);
        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "conf", 1, sensor_msgs::msg::PointField::FLOAT32);

        float *cloudPtr = reinterpret_cast<float *>(&msgPointCloud->data[0]);
        ::memcpy(cloudPtr, data->xyzcPoints, 4 * sizeof(float) * numPoints);

        m_pubCloud[curIdx]->publish(std::move(msgPointCloud));
    }

    if (m_isPubDepth) {
        // Create Depth Image message
        sensor_msgs::msg::Image::UniquePtr msgDepthImage(new sensor_msgs::msg::Image);

        msgDepthImage->header = header;
        msgDepthImage->width = data->width;
        msgDepthImage->height = data->height;
        msgDepthImage->is_bigendian = false;
        msgDepthImage->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        msgDepthImage->step = static_cast<uint32_t>(sizeof(float) * data->width);
        msgDepthImage->data.resize(sizeof(float) * numPoints);

        float *iterDepth = (float *)&msgDepthImage->data[0];

        // Iterate over all the points we received in the callback
        for (auto i = 0u; i < numPoints; ++i) {
            *iterDepth++ = data->xyzcPoints[i * 4 + 2];
        }
        m_pubDepth[curIdx]->publish(std::move(msgDepthImage));
    }

    // Publish with CameraInfo
    sensor_msgs::msg::CameraInfo::UniquePtr msgCameraInfo(new sensor_msgs::msg::CameraInfo);
    *msgCameraInfo = m_cameraInfo;
    msgCameraInfo->header = header;
    msgCameraInfo->height = data->height;
    msgCameraInfo->width = data->width;
    m_pubCameraInfo->publish(std::move(msgCameraInfo));
}

void CameraNode::onNewData(const royale::IRImage *data) {
    auto curIdx = m_streamIdx[data->streamId];

    std_msgs::msg::Header header;
    header.frame_id = "pmd_royale_ros_camera_node_optical_frame";
    header.stamp = rclcpp::Time(
        (chrono::duration_cast<chrono::nanoseconds>(chrono::microseconds(data->timestamp))).count());

    auto numPoints = data->getNumPoints();

    // Create Depth Image message
    sensor_msgs::msg::Image::UniquePtr msgGrayImage(new sensor_msgs::msg::Image);

    msgGrayImage->header = header;
    msgGrayImage->width = data->width;
    msgGrayImage->height = data->height;
    msgGrayImage->is_bigendian = false;
    msgGrayImage->encoding = sensor_msgs::image_encodings::MONO8;
    msgGrayImage->step = static_cast<uint32_t>(data->width);
    msgGrayImage->data.resize(numPoints);

    ::memcpy(&msgGrayImage->data[0], data->data, numPoints);

    m_pubGray[curIdx]->publish(std::move(msgGrayImage));

    // Publish with CameraInfo
    sensor_msgs::msg::CameraInfo::UniquePtr msgCameraInfo(new sensor_msgs::msg::CameraInfo);
    *msgCameraInfo = m_cameraInfo;
    msgCameraInfo->header = header;
    msgCameraInfo->height = data->height;
    msgCameraInfo->width = data->width;
    m_pubCameraInfo->publish(std::move(msgCameraInfo));
}

void CameraNode::onNewExposure(const uint32_t exposureTime, const royale::StreamId streamId) {
    auto curIdx = m_streamIdx[streamId];
    if (m_exposureTime[curIdx] == exposureTime) {
        return;
    }

    m_exposureTime[curIdx] = exposureTime;

    try {
        this->set_parameter(rclcpp::Parameter("exposure_time_" + std::to_string(curIdx), (int)m_exposureTime[curIdx]));
    } catch (std::exception &exception) {
        RCLCPP_INFO(this->get_logger(), "Caught exception in onNewExposure callback: %s", exception.what());
    }
}

rcl_interfaces::msg::SetParametersResult CameraNode::onSetParameters(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (auto &parameter : parameters) {
        if (!result.successful) {
            break;
        }
        if (parameter.get_name() == "usecase" && parameter.get_type() == rclcpp::PARAMETER_STRING) {
            result.successful = setUseCase(parameter.as_string());
        } else if (parameter.get_name().find("exposure_time_") == 0 && parameter.get_type() == rclcpp::PARAMETER_INTEGER) {
            auto streamIdxStr = parameter.get_name().substr(strlen("exposure_time_"));
            auto streamIdx = stoi(streamIdxStr);

            if (!m_isAutoExposureEnabled[streamIdx]) {
                StreamId streamId = 0;
                bool found = false;
                for (auto curIdx : m_streamIdx) {
                    if (curIdx.second == streamIdx) {
                        streamId = curIdx.first;
                        found = true;
                    }
                }
                if (found) {
                    result.successful = setExposureTime((int)parameter.as_int(), streamId);
                }
            }
        } else if (parameter.get_name().find("auto_exposure_") == 0 && parameter.get_type() == rclcpp::PARAMETER_BOOL) {
            auto streamIdxStr = parameter.get_name().substr(strlen("auto_exposure_"));
            auto streamIdx = stoi(streamIdxStr);
            StreamId streamId = 0;
            bool found = false;
            for (auto curIdx : m_streamIdx) {
                if (curIdx.second == streamIdx) {
                    streamId = curIdx.first;
                    found = true;
                }
            }
            if (found) {
                result.successful = enableAutoExposure(parameter.as_bool(), streamId);
            }
        }
    }

    return result;
}

void CameraNode::onParametersSetEvent(const rcl_interfaces::msg::ParameterEvent &event) {
    auto params = rclcpp::ParameterEventHandler::get_parameters_from_event(event);
    for (auto &param : params) {
        if (param.get_name() == "usecase") {
            initUseCase();

            if (m_cameraDevice->registerExposureListener(this) != CameraStatus::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Couldn't register exposure listener!");
                return;
            }
            m_cameraDevice->startCapture();
        }
    }
}

bool CameraNode::setCameraInfo() {
    LensParameters lensParams;
    if ((m_cameraDevice->getLensParameters(lensParams) == CameraStatus::SUCCESS)) {
        if (lensParams.distortionRadial.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "Unknown distortion model!");
            return false;
        } else {
            m_cameraInfo.distortion_model = "plumb_bob";
            m_cameraInfo.d.resize(5);
            m_cameraInfo.d[0] = lensParams.distortionRadial[0];
            m_cameraInfo.d[1] = lensParams.distortionRadial[1];
            m_cameraInfo.d[2] = lensParams.distortionTangential.first;
            m_cameraInfo.d[3] = lensParams.distortionTangential.second;
            m_cameraInfo.d[4] = lensParams.distortionRadial[2];
        }

        m_cameraInfo.k[0] = lensParams.focalLength.first;
        m_cameraInfo.k[1] = 0;
        m_cameraInfo.k[2] = lensParams.principalPoint.first;
        m_cameraInfo.k[3] = 0;
        m_cameraInfo.k[4] = lensParams.focalLength.second;
        m_cameraInfo.k[5] = lensParams.principalPoint.second;
        m_cameraInfo.k[6] = 0;
        m_cameraInfo.k[7] = 0;
        m_cameraInfo.k[8] = 1;

        m_cameraInfo.r[0] = 1;
        m_cameraInfo.r[1] = 0;
        m_cameraInfo.r[2] = 0;
        m_cameraInfo.r[3] = 0;
        m_cameraInfo.r[4] = 1;
        m_cameraInfo.r[5] = 0;
        m_cameraInfo.r[6] = 0;
        m_cameraInfo.r[7] = 0;
        m_cameraInfo.r[8] = 1;

        m_cameraInfo.p[0] = lensParams.focalLength.first;
        m_cameraInfo.p[1] = 0;
        m_cameraInfo.p[2] = lensParams.principalPoint.first;
        m_cameraInfo.p[3] = 0;
        m_cameraInfo.p[4] = 0;
        m_cameraInfo.p[5] = lensParams.focalLength.second;
        m_cameraInfo.p[6] = lensParams.principalPoint.second;
        m_cameraInfo.p[7] = 0;
        m_cameraInfo.p[8] = 0;
        m_cameraInfo.p[9] = 0;
        m_cameraInfo.p[10] = 1;
        m_cameraInfo.p[11] = 0;

        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Couldn't get lens parameters!");
        return false;
    }
}

bool CameraNode::setUseCase(const std::string &useCase) {
    if (!m_recording_file.empty()) {
        m_cameraDevice->stopRecording();
    }
    m_cameraDevice->stopCapture();
    m_cameraDevice->unregisterExposureListener();
    auto result = m_cameraDevice->setUseCase(useCase);
    if (result != royale::CameraStatus::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set usecase: %s. Result = %d", useCase.c_str(), (int)result);
        return false;
    }

    return true;
}

bool CameraNode::setExposureTime(int exposureTime, royale::StreamId streamId) {
    if (m_isAutoExposureEnabled[m_streamIdx[streamId]]) {
        return true;
    }

    RCLCPP_INFO(this->get_logger(), "Setting exposure: %d %d", (int)exposureTime, streamId);
    int tries = 5;
    CameraStatus ret;
    do {
        ret = m_cameraDevice->setExposureTime(exposureTime, streamId);
        if (ret == CameraStatus::DEVICE_IS_BUSY) {
            this_thread::sleep_for(chrono::milliseconds(200));
            tries--;
        } else if (ret == CameraStatus::EXPOSURE_MODE_INVALID) {
            // we tried to set an exposure time even though auto exposure is activated
            return true;
        } else if (ret != CameraStatus::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Error setting exposure time: %d", (int)ret);
            return false;
        }
    } while (tries > 0 && ret == CameraStatus::DEVICE_IS_BUSY);

    return ret == CameraStatus::SUCCESS;
}

bool CameraNode::enableAutoExposure(bool enable, royale::StreamId streamId) {
    RCLCPP_INFO(this->get_logger(), "Setting auto exposure: %d %d", (int)enable, streamId);
    auto result = m_cameraDevice->setExposureMode(enable ? royale::ExposureMode::AUTOMATIC : royale::ExposureMode::MANUAL, streamId);
    if (result != royale::CameraStatus::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Error setting auto exposure: %d", (int)enable);
        return false;
    }
    m_isAutoExposureEnabled[m_streamIdx[streamId]] = enable;
    return result == CameraStatus::SUCCESS;
}

void CameraNode::initUseCase() {
    Vector<StreamId> streamIds;
    if (m_cameraDevice->getStreams(streamIds) != CameraStatus::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't retrieve streams!");
        return;
    }
    m_streamIdx.clear();
    for (auto i = 0u; i < streamIds.size(); ++i) {
        m_streamIdx[streamIds[i]] = i;
    }

    for (auto i = 0u; i < streamIds.size(); ++i) {
        ExposureMode expoMode;
        m_cameraDevice->getExposureMode(expoMode, streamIds[i]);
        m_isAutoExposureEnabled[i] = (expoMode == ExposureMode::AUTOMATIC);

        this->set_parameter(rclcpp::Parameter("auto_exposure_" + std::to_string(i), m_isAutoExposureEnabled[i]));

        royale::Pair<uint32_t, uint32_t> exposureLimits;
        m_cameraDevice->getExposureLimits(exposureLimits, streamIds[i]);

        rcl_interfaces::msg::ParameterDescriptor exposureParamDescriptor = this->describe_parameter("exposure_time_" + std::to_string(i));
        exposureParamDescriptor.integer_range.clear();
        rcl_interfaces::msg::IntegerRange exposureTimeRange;
        exposureTimeRange.from_value = exposureLimits.first;
        exposureTimeRange.to_value = exposureLimits.second;
        exposureTimeRange.step = 1;
        exposureParamDescriptor.integer_range.push_back(exposureTimeRange);
        this->undeclare_parameter("exposure_time_" + std::to_string(i));
        this->declare_parameter("exposure_time_" + std::to_string(i), (int)exposureLimits.second, exposureParamDescriptor);
    }
}

void CameraNode::updateDataListeners() {
    m_isPubCloud = false;
    m_isPubDepth = false;
    m_isPubGray = false;

    for (auto i = 0u; i < ROYALE_ROS_MAX_STREAMS; ++i) {
        m_isPubCloud |= m_pubCloud[i]->get_subscription_count() > 0 || m_pubCloud[i]->get_intra_process_subscription_count() > 0;
        m_isPubDepth |= m_pubDepth[i]->get_subscription_count() > 0 || m_pubDepth[i]->get_intra_process_subscription_count() > 0;
        m_isPubGray |= m_pubGray[i]->get_subscription_count() > 0 || m_pubGray[i]->get_intra_process_subscription_count() > 0;
    }

    bool shouldRegisterPCListener = m_isPubCloud || m_isPubDepth;

    if (!m_registeredPCListener && shouldRegisterPCListener) {
        if (m_cameraDevice->registerPointCloudListener(this) == CameraStatus::SUCCESS) {
            m_registeredPCListener = true;
            RCLCPP_DEBUG(this->get_logger(), "Registered pointcloud data listener!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Couldn't register pointcloud data listener!");
        }
    } else if (m_registeredPCListener && !shouldRegisterPCListener) {
        if (m_cameraDevice->unregisterPointCloudListener() == CameraStatus::SUCCESS) {
            m_registeredPCListener = false;
            RCLCPP_DEBUG(this->get_logger(), "Unregistered pointcloud data listener!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Couldn't unregister pointcloud data listener!");
        }
    }

    if (!m_registeredIRListener && m_isPubGray) {
        if (m_cameraDevice->registerIRImageListener(this) == CameraStatus::SUCCESS) {
            m_registeredIRListener = true;
            RCLCPP_DEBUG(this->get_logger(), "Registered IR data listener!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Couldn't register IR data listener!");
        }
    } else if (m_registeredIRListener && !m_isPubGray) {
        if (m_cameraDevice->unregisterIRImageListener() == CameraStatus::SUCCESS) {
            m_registeredIRListener = false;
            RCLCPP_DEBUG(this->get_logger(), "Unregistered IR data listener!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Couldn't unregister IR data listener!");
        }
    }
}

void CameraNode::setProcParams(const std_msgs::msg::String::SharedPtr parameters, uint32_t streamIdx) {
    stringstream ss(parameters->data);
    string word;
    vector<string> params;
    while (ss >> word) {
        params.push_back(word);
    }
    if (params.size() != 2u) {
        // we should get a parameter name and a value
        RCLCPP_ERROR(this->get_logger(), "Invalid parameter : %s", ss.str().c_str());
        return;
    }

    royale::Variant value;
    royale::VariantType paramType;
    if (getProcessingFlagType(params[0], paramType)) {
        if (paramType == royale::VariantType::Bool) {
            bool bvalue;
            stringstream s2(params[1]);
            s2 >> std::boolalpha >> bvalue;
            value.setBool(bvalue);
        } else if (paramType == royale::VariantType::Int) {
            int32_t ivalue;
            stringstream s2(params[1]);
            s2 >> ivalue;
            value.setInt(static_cast<int>(ivalue));
        } else if (paramType == royale::VariantType::Float) {
            float fvalue;
            stringstream s2(params[1]);
            s2 >> fvalue;
            value.setFloat(fvalue);
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Processing parameter unknown : %s", params[0].c_str());
        return;
    }
    StreamId streamId = 0;
    for (auto curIdx : m_streamIdx) {
        if (curIdx.second == streamIdx) {
            streamId = curIdx.first;
        }
    }

    royale::Vector<royale::Pair<royale::String, royale::Variant>> newParam({{params[0], value}});
    auto ret = m_cameraDevice->setProcessingParameters(newParam, streamId);

    if (ret == CameraStatus::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Successfully set parameter : %d %s", streamId, parameters->data.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Error setting parameter : %d %s", streamId, parameters->data.c_str());
    }
}

} // namespace pmd_royale_ros_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pmd_royale_ros_driver::CameraNode)