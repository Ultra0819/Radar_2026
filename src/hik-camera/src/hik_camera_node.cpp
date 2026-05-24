#include "MvCameraControl.h"
#include "hik_camera/gpu_image_converter.hpp"
// #include "CameraParams.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <chrono>
#include <cstring>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <thread>

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");
    RCLCPP_INFO(this->get_logger(), "SDK version: %x", MV_CC_GetSDKVersion());

    MV_CC_DEVICE_INFO_LIST device_list;
    // enum device
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "No camera found!");
      RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    // 根据序列号启动确认
    bool enable_DeviceSerialNumber = this->declare_parameter("use_DeviceSerialNumber", false);
    if (!enable_DeviceSerialNumber) {
      RCLCPP_INFO(this->get_logger(), "target_DeviceSerialNumber: None");
      MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
      nRet = MV_CC_OpenDevice(camera_handle_);
      MVCC_STRINGVALUE stParam;
      MV_CC_GetStringValue(camera_handle_, "DeviceSerialNumber", &stParam);
      RCLCPP_INFO(this->get_logger(), "DeviceSerialNumber: %s", stParam.chCurValue);
    } else {
      std::string SerialNumber =
        this->declare_parameter("DeviceSerialNumber", "DA1564615");  // DA1564615  DA1041822
      for (unsigned int i = 0; i < device_list.nDeviceNum; i++) {
        MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[i]);
        nRet = MV_CC_OpenDevice(camera_handle_);
        if (nRet != MV_OK) continue;
        MVCC_STRINGVALUE stParam;
        MV_CC_GetStringValue(camera_handle_, "DeviceSerialNumber", &stParam);
        RCLCPP_INFO(this->get_logger(), "DeviceSerialNumber: %s", stParam.chCurValue);

        if (stParam.chCurValue == SerialNumber)
          break;
        else
          MV_CC_CloseDevice(camera_handle_);
      }
    }

    // Get camera infomation
    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

    // Init convert param
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "camera_image", qos);

    declareParameters();  // 设置相机参数
    initGpuConverter();

    nRet = MV_CC_StartGrabbing(camera_handle_);
    RCLCPP_WARN(this->get_logger(), "MV_CC_StartGrabbing! nRet: [%x]", nRet);

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
      this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
      MV_FRAME_OUT out_frame;

      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      // Set image message header
      // image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.header.frame_id = frame_id_;
      image_msg_.encoding = "rgb8";
      // image_msg_.encoding = "bayer_rggb8";

      while (rclcpp::ok()) {
        nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
        auto now = this->now();
        if (MV_OK == nRet) {
          if (!convertFrameToRgb8(out_frame)) {
            MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
            continue;
          }

          image_msg_.header.stamp = now;

          camera_info_msg_.header = image_msg_.header;
          camera_pub_.publish(image_msg_, camera_info_msg_);

          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
          fail_conut_ = 0;
        } else {
          RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
          nRet = MV_CC_StopGrabbing(camera_handle_);
          RCLCPP_WARN(this->get_logger(), "MV_CC_StopGrabbing! nRet: [%x]", nRet);
          nRet = MV_CC_StartGrabbing(camera_handle_);
          RCLCPP_WARN(this->get_logger(), "MV_CC_StartGrabbing! nRet: [%x]", nRet);
          fail_conut_++;
        }

        if (fail_conut_ > 5) {
          RCLCPP_FATAL(this->get_logger(), "Camera failed!");
          rclcpp::shutdown();
        }
      }
    }};
  }

  ~HikCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:
  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    param_desc.floating_point_range.resize(1);

    // Acquisition frame rate
    param_desc.description = "Acquisition frame rate in Hz";
    MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &f_value);
    param_desc.floating_point_range[0].from_value = f_value.fMin;
    param_desc.floating_point_range[0].to_value = f_value.fMax;
    double acquisition_frame_rate =
      this->declare_parameter("acquisition_frame_rate", 59.0, param_desc);
    MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
    MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", acquisition_frame_rate);
    RCLCPP_INFO(this->get_logger(), "Acquisition frame rate: %f", acquisition_frame_rate);

    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    param_desc.floating_point_range[0].from_value = f_value.fMin;
    param_desc.floating_point_range[0].to_value = f_value.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 5000.0, param_desc);

    //关闭自动曝光
    nRet = MV_CC_SetEnumValueByString(camera_handle_, "ExposureAuto", "Off");
    if (nRet != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to turn off ExposureAuto, status = %x", nRet);
    }

    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // Gain
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    param_desc.floating_point_range[0].from_value = f_value.fMin;
    param_desc.floating_point_range[0].to_value = f_value.fMax;
    double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);

    //关闭自动增益
    nRet = MV_CC_SetEnumValueByString(camera_handle_, "GainAuto", "Off");
    if (nRet != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to turn off GainAuto, status = %x", nRet);
    }

    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);

    // ADC Bit Depth
    param_desc.description = "ADC Bit Depth";
    param_desc.additional_constraints = "Supported values: ADCBitDepth_8, ADCBitDepth_12";
    std::string adc_bit_depth =
      this->declare_parameter("adc_bit_depth", "ADCBitDepth_8", param_desc);
    nRet = MV_CC_SetEnumValueByString(camera_handle_, "ADCBitDepth", adc_bit_depth.c_str());
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "ADC Bit Depth set to %s", adc_bit_depth.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set ADC Bit Depth, status = %d", nRet);
    }

    // Pixel format
    param_desc.description = "Pixel Format";
    std::string pixel_format = this->declare_parameter("pixel_format", "BayerRG8", param_desc);
    nRet = MV_CC_SetEnumValueByString(camera_handle_, "PixelFormat", pixel_format.c_str());
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Pixel Format set to %s", pixel_format.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set Pixel Format, status = %d", nRet);
    }

    frame_id_ = this->declare_parameter("target_frame", "camera_optical_frame");
    RCLCPP_INFO(this->get_logger(), "frame_id_: %s", frame_id_.c_str());

    use_gpu_converter_ = this->declare_parameter("use_gpu_converter", true);
    RCLCPP_INFO(
      this->get_logger(), "GPU Bayer converter: %s", use_gpu_converter_ ? "enabled" : "disabled");
  }

  void initGpuConverter()
  {
    if (!use_gpu_converter_) {
      RCLCPP_INFO(this->get_logger(), "Using SDK CPU pixel conversion path.");
      return;
    }

    gpu_converter_ = std::make_unique<GpuImageConverter>();
    if (gpu_converter_->available()) {
      RCLCPP_INFO(this->get_logger(), "CUDA Bayer8 -> RGB8 converter is ready.");
    } else {
      RCLCPP_WARN(
        this->get_logger(), "CUDA converter unavailable: %s. Falling back to SDK CPU conversion.",
        gpu_converter_->lastError().c_str());
    }
  }

  bool convertFrameToRgb8(const MV_FRAME_OUT & out_frame)
  {
    const uint32_t width = out_frame.stFrameInfo.nWidth;
    const uint32_t height = out_frame.stFrameInfo.nHeight;
    const std::size_t rgb_size = static_cast<std::size_t>(width) * height * 3;
    const uint32_t pixel_type = static_cast<uint32_t>(out_frame.stFrameInfo.enPixelType);
    const auto * src_data = reinterpret_cast<const uint8_t *>(out_frame.pBufAddr);

    image_msg_.height = height;
    image_msg_.width = width;
    image_msg_.encoding = "rgb8";
    image_msg_.step = width * 3;
    image_msg_.data.resize(rgb_size);

    if (pixel_type == PixelType_Gvsp_RGB8_Packed) {
      if (out_frame.stFrameInfo.nFrameLen < rgb_size) {
        RCLCPP_WARN(
          this->get_logger(), "RGB frame is too small: len=%u expected=%zu",
          out_frame.stFrameInfo.nFrameLen, rgb_size);
        return false;
      }
      std::memcpy(image_msg_.data.data(), src_data, rgb_size);
      return true;
    }

    if (use_gpu_converter_ && gpu_converter_ != nullptr && gpu_converter_->available()) {
      if (GpuImageConverter::isSupportedBayer8(pixel_type)) {
        if (gpu_converter_->convertBayer8ToRgb8(
              src_data, out_frame.stFrameInfo.nFrameLen, width, height, pixel_type,
              image_msg_.data.data(), image_msg_.data.size())) {
          return true;
        }

        if (!gpu_failure_warned_) {
          RCLCPP_WARN(
            this->get_logger(), "CUDA conversion failed: %s. Falling back to SDK CPU conversion.",
            gpu_converter_->lastError().c_str());
          gpu_failure_warned_ = true;
        }
      } else if (!gpu_unsupported_warned_) {
        RCLCPP_WARN(
          this->get_logger(),
          "GPU converter only handles 8-bit Bayer frames, got pixel type 0x%x. "
          "Falling back to SDK CPU conversion.",
          pixel_type);
        gpu_unsupported_warned_ = true;
      }
    }

    convert_param_.nWidth = width;
    convert_param_.nHeight = height;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    convert_param_.pDstBuffer = image_msg_.data.data();
    convert_param_.nDstBufferSize = image_msg_.data.size();
    convert_param_.pSrcData = out_frame.pBufAddr;
    convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
    convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

    int convert_ret = MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
    if (convert_ret != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Convert pixel type failed! nRet: [%x]", convert_ret);
      return false;
    }

    return true;
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        // exposure_time declared as double
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gain") {
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "use_gpu_converter") {
        result.successful = false;
        result.reason = "use_gpu_converter must be set before node startup";
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  sensor_msgs::msg::Image image_msg_;

  image_transport::CameraPublisher camera_pub_;

  int nRet = MV_OK;
  void * camera_handle_ = nullptr;
  MV_IMAGE_BASIC_INFO img_info_;

  MV_CC_PIXEL_CONVERT_PARAM convert_param_;
  std::unique_ptr<GpuImageConverter> gpu_converter_;
  bool use_gpu_converter_ = true;
  bool gpu_failure_warned_ = false;
  bool gpu_unsupported_warned_ = false;

  std::string camera_name_;
  std::string frame_id_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int fail_conut_ = 0;
  std::thread capture_thread_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
