// -*-c++-*--------------------------------------------------------------------
// Simple standalone node to reset one or more FLIR/Spinnaker cameras
//
// This node uses the existing SpinnakerWrapper to issue the "DeviceReset"
// GenICam command to the specified cameras (by serial number) and then exits.
//
// Parameters:
//   serial_numbers (string array): list of camera serials to reset.
//                                  Defaults to the two cameras you requested.
//   wait_after_reset_sec (double): optional delay after each reset, in seconds.
//

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <spinnaker_camera_driver/spinnaker_wrapper.hpp>

using namespace std::chrono_literals;

namespace spinnaker_camera_driver
{

class CameraResetNode : public rclcpp::Node
{
public:
  CameraResetNode()
  : rclcpp::Node("camera_reset_node")
  {
    // Default serials as requested by the user.
    const std::vector<std::string> default_serials = {"24462485", "25040024"};

    serial_numbers_ = this->declare_parameter<std::vector<std::string>>(
      "serial_numbers", default_serials);

    wait_after_reset_ = this->declare_parameter<double>("wait_after_reset_sec", 0.0);

    RCLCPP_INFO(
      get_logger(), "Camera reset node starting, %zu serial(s) configured",
      serial_numbers_.size());

    for (const auto & sn : serial_numbers_) {
      RCLCPP_INFO(get_logger(), "  serial: %s", sn.c_str());
    }

    run();
  }

private:
  void run()
  {
    try {
      SpinnakerWrapper wrapper;
      wrapper.setDebug(false);

      wrapper.refreshCameraList();
      auto available = wrapper.getSerialNumbers();

      if (available.empty()) {
        RCLCPP_WARN(get_logger(), "No cameras detected by Spinnaker!");
      } else {
        std::string list;
        for (size_t i = 0; i < available.size(); i++) {
          list += available[i];
          if (i + 1 < available.size()) {
            list += ", ";
          }
        }
        RCLCPP_INFO(get_logger(), "Detected camera serials: [%s]", list.c_str());
      }

      for (const auto & serial : serial_numbers_) {
        if (!serial_present(serial, available)) {
          RCLCPP_WARN(
            get_logger(),
            "Camera with serial %s not found in current list - skipping reset!",
            serial.c_str());
          continue;
        }

        RCLCPP_INFO(get_logger(), "Initializing camera with serial %s", serial.c_str());
        if (!wrapper.initCamera(serial)) {
          RCLCPP_ERROR(
            get_logger(), "Failed to initialize camera %s, skipping reset",
            serial.c_str());
          continue;
        }

        RCLCPP_INFO(get_logger(), "Executing DeviceReset on camera %s", serial.c_str());
        const std::string res = wrapper.execute("DeviceControl/DeviceReset");
        if (res != "OK") {
          RCLCPP_ERROR(
            get_logger(),
            "DeviceReset command returned '%s' for camera %s",
            res.c_str(), serial.c_str());
        } else {
          RCLCPP_INFO(
            get_logger(), "Camera %s has been reset (DeviceReset OK)",
            serial.c_str());
        }

        if (!wrapper.deInitCamera()) {
          RCLCPP_WARN(
            get_logger(), "DeInit failed or camera already de-initialized for %s",
            serial.c_str());
        }

        if (wait_after_reset_ > 0.0) {
          const auto wait_ms =
            static_cast<int64_t>(wait_after_reset_ * 1000.0);
          RCLCPP_INFO(
            get_logger(), "Waiting %.3f seconds after reset of %s",
            wait_after_reset_, serial.c_str());
          std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
        }
      }

      RCLCPP_INFO(get_logger(), "Camera reset node finished, shutting down.");

    } catch (const SpinnakerWrapper::Exception & e) {
      RCLCPP_ERROR(
        get_logger(), "SpinnakerWrapper exception during camera reset: %s",
        e.what());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_logger(), "Standard exception during camera reset: %s",
        e.what());
    }
  }

  static bool serial_present(
    const std::string & serial,
    const std::vector<std::string> & available)
  {
    return std::find(available.begin(), available.end(), serial) != available.end();
  }

  std::vector<std::string> serial_numbers_;
  double wait_after_reset_{0.0};
};

}  // namespace spinnaker_camera_driver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<spinnaker_camera_driver::CameraResetNode>();
    // Node does all its work in the constructor; nothing to spin.
  }
  rclcpp::shutdown();
  return 0;
}



