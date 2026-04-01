/*
The MIT License (MIT)
Copyright (c) 2019 Techno Road Inc.
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <tf2_ros/transform_broadcaster.h>

#include <adi_imu_tr_driver_ros2/srv/simple_cmd.hpp>
#include <chrono>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "adis_rcv_bin.h"

using SimpleCmd = adi_imu_tr_driver_ros2::srv::SimpleCmd;
using namespace std::chrono_literals;

class ImuNodeRcvBin : public rclcpp::Node
{
 public:
  explicit ImuNodeRcvBin(const rclcpp::NodeOptions& op) : Node("adi_rcv_bin_node", op)
  {
    InitParam();

    std::chrono::milliseconds ms(static_cast<int>(1.0 / rate_ * 1000));

    updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    updater_->add("imu", this, &ImuNodeRcvBin::Diagnostic);

    imu_data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 1);
    tf_br_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 1);
    cmd_server_ = this->create_service<SimpleCmd>(
        "/imu/cmd_srv", std::bind(&ImuNodeRcvBin::CmdCb, this, std::placeholders::_1,
                                  std::placeholders::_2, std::placeholders::_3));

    Prepare();

    timer_ = this->create_wall_timer(ms, std::bind(&ImuNodeRcvBin::Spin, this));
  }

  ~ImuNodeRcvBin()
  {
    RCLCPP_INFO(this->get_logger(), "Destructor was called!");
    imu_.StopTelemetry();
    imu_.Close();
  }

  void Prepare()
  {
    std::chrono::seconds sec(1);
    rclcpp::Rate r(sec);

    // Open serial port
    while (rclcpp::ok() && !imu_.Open(device_)) {
      RCLCPP_WARN(this->get_logger(), "Keep trying to open [%s] in 1 second period...",
                  device_.c_str());
      r.sleep();
    }

    // Stop any ongoing telemetry and flush
    imu_.StopTelemetry();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read settings to get sensitivity and product info
    int retry = 3;
    bool settings_ok = false;
    for (int i = 0; i < retry && rclcpp::ok(); i++) {
      if (imu_.ReadSettings()) {
        settings_ok = true;
        break;
      }
      RCLCPP_WARN(this->get_logger(), "Failed to read settings, retry %d/%d...", i + 1, retry);
      r.sleep();
    }

    if (settings_ok) {
      auto& s = imu_.GetSettings();
      RCLCPP_INFO(this->get_logger(), "Product: %s", imu_.GetProductIdStr().c_str());
      RCLCPP_INFO(this->get_logger(), "Build date: %u", s.build_date);
      RCLCPP_INFO(this->get_logger(), "Sample rate: %u Hz", s.sample_rate);
      RCLCPP_INFO(this->get_logger(), "Filter: %u, Grav corr: %u", s.filter_select,
                  s.grav_corr_en);
      RCLCPP_INFO(this->get_logger(), "Accel sensitivity: %lu, Gyro sensitivity: %lu",
                  static_cast<unsigned long>(s.accl_sensitivity),
                  static_cast<unsigned long>(s.gyro_sensitivity));
      updater_->setHardwareID(imu_.GetProductIdStr());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Could not read settings! Sensitivity values may be wrong.");
      updater_->setHardwareID("UNKNOWN");
    }

    // Start telemetry
    if (!imu_.StartTelemetry()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start telemetry!");
    }
  }

 private:
  AdisRcvBin imu_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_br_;
  rclcpp::Service<SimpleCmd>::SharedPtr cmd_server_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;

  std::string device_;
  std::string frame_id_;
  std::string parent_id_;
  double rate_;
  int cant_rcv_cnt_;

  void InitParam()
  {
    device_ = "/dev/ttyACM0";
    frame_id_ = "imu";
    parent_id_ = "odom";
    rate_ = 100.0;

    std::string key = "device";
    if (this->get_parameter(key, device_)) {
      RCLCPP_INFO(this->get_logger(), "%s: %s", key.c_str(), device_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not get param %s. Set default value: %s", key.c_str(),
                  device_.c_str());
    }

    key = "frame_id";
    if (this->get_parameter(key, frame_id_)) {
      RCLCPP_INFO(this->get_logger(), "%s: %s", key.c_str(), frame_id_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not get param %s. Set default value: %s", key.c_str(),
                  frame_id_.c_str());
    }

    key = "parent_id";
    if (this->get_parameter(key, parent_id_)) {
      RCLCPP_INFO(this->get_logger(), "%s: %s", key.c_str(), parent_id_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not get param %s. Set default value: %s", key.c_str(),
                  parent_id_.c_str());
    }

    key = "rate";
    if (this->get_parameter(key, rate_)) {
      RCLCPP_INFO(this->get_logger(), "%s: %.1f", key.c_str(), rate_);
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not get param %s. Set default value: %.1f",
                  key.c_str(), rate_);
    }

    cant_rcv_cnt_ = 0;
  }

  void CmdCb(const std::shared_ptr<rmw_request_id_t> req_header,
             const std::shared_ptr<SimpleCmd::Request> req,
             const std::shared_ptr<SimpleCmd::Response> res)
  {
    (void)req_header;
    constexpr size_t kDataSize = 8;
    res->is_ok = true;

    if (req->cmd.empty()) {
      res->is_ok = false;
      res->msg = "Empty command.";
      return;
    }

    // Parse command ID from hex string (e.g. "0x70")
    uint8_t cmd_id = 0;
    try {
      cmd_id = static_cast<uint8_t>(std::stoul(req->cmd, nullptr, 0));
    } catch (const std::exception& e) {
      res->is_ok = false;
      res->msg = "Invalid command ID: " + req->cmd;
      return;
    }

    // Build 8-byte data from args, zero-padded
    uint8_t data[kDataSize] = {};
    for (size_t i = 0; i < req->args.size() && i < kDataSize; i++) {
      try {
        data[i] = static_cast<uint8_t>(std::stoul(req->args[i], nullptr, 0));
      } catch (const std::exception& e) {
        res->is_ok = false;
        res->msg = "Invalid arg[" + std::to_string(i) + "]: " + req->args[i];
        return;
      }
    }

    bool ok = imu_.SendCommand(cmd_id, data, kDataSize);

    if (!ok) {
      res->is_ok = false;
      res->msg = "Command failed: " + req->cmd;
    } else {
      res->msg = "OK";
    }
  }

  void Diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if (cant_rcv_cnt_ >= 1) {
      stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                    "Data cannot be received for more than 1 second.");
    } else {
      stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
    }
  }

  void PubImuData()
  {
    auto data = std::make_shared<sensor_msgs::msg::Imu>();

    data->header.frame_id = frame_id_;
    data->header.stamp = this->now();

    double acc[3];
    double gyro[3];
    double quat[4];
    imu_.GetAccSI(acc);
    imu_.GetGyroSI(gyro);
    imu_.GetQuat(quat);

    // Orientation from quaternion
    data->orientation.w = quat[0];
    data->orientation.x = quat[1];
    data->orientation.y = quat[2];
    data->orientation.z = quat[3];

    // Linear acceleration
    data->linear_acceleration.x = acc[0];
    data->linear_acceleration.y = acc[1];
    data->linear_acceleration.z = acc[2];

    // Angular velocity
    data->angular_velocity.x = gyro[0];
    data->angular_velocity.y = gyro[1];
    data->angular_velocity.z = gyro[2];

    imu_data_pub_->publish(*data);
  }

  void BroadcastImuPose()
  {
    tf2_msgs::msg::TFMessage tf_msg;
    geometry_msgs::msg::TransformStamped tff;

    double quat[4];
    imu_.GetQuat(quat);

    tff.transform.translation.x = 0;
    tff.transform.translation.y = 0;
    tff.transform.translation.z = 0;

    tff.transform.rotation.w = quat[0];
    tff.transform.rotation.x = quat[1];
    tff.transform.rotation.y = quat[2];
    tff.transform.rotation.z = quat[3];

    tff.child_frame_id = frame_id_;
    tff.header.frame_id = parent_id_;
    tff.header.stamp = this->now();

    tf_msg.transforms.push_back(tff);
    tf_br_->publish(tf_msg);
  }

  void PrintErrorCode(int code)
  {
    switch (code) {
      case kImuBinErrCantRcvData:
        RCLCPP_ERROR(this->get_logger(), "Cannot read data from port");
        break;
      case kImuBinErrCouldNotFindPkt:
        RCLCPP_ERROR(this->get_logger(), "Cannot find packet");
        break;
      case kImuBinErrInvalidData:
        RCLCPP_ERROR(this->get_logger(), "Invalid data");
        break;
      case kImuBinErrChecksum:
        RCLCPP_ERROR(this->get_logger(), "Checksum error");
        break;
      default:
        break;
    }
  }

  void Spin()
  {
    if (imu_.GetState() != AdisRcvBin::State::RUNNING) return;

    int res = imu_.UpdateTelemetry();
    if (res == kImuBinOk) {
      PubImuData();
      BroadcastImuPose();
      cant_rcv_cnt_ = 0;
    } else {
      if (res == kImuBinErrCantRcvData) cant_rcv_cnt_++;
      PrintErrorCode(res);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto imu_node = std::make_shared<ImuNodeRcvBin>(options);

  rclcpp::spin(imu_node);
  rclcpp::shutdown();
  return 0;
}
