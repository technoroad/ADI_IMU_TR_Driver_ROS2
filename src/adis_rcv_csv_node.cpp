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
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // for use tf2::Quaternion
#include <diagnostic_updater/diagnostic_updater.hpp>


#include "adis_rcv_csv.h"


using namespace std::chrono_literals;

class ImuNodeRcvCsv : public rclcpp::Node {
public:

AdisRcvCsv imu_;
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;
rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_br_;
std::unique_ptr<diagnostic_updater::Updater> updater_;

std::string device_;
std::string frame_id_;
std::string parent_id_;
double rate_;
int cant_rcv_cnt_;

explicit ImuNodeRcvCsv(const rclcpp::NodeOptions& op)
  : Node("adi_rcv_csv_node", op) {

  InitParam();

  std::chrono::milliseconds ms((int)(1.0/rate_));

  updater_ = std::make_unique<diagnostic_updater::Updater>(this);
  updater_->add("imu", this, &ImuNodeRcvCsv::Diagnostic);

  InitDevice();

  // Data publisher
  imu_data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 1);
  tf_br_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 1);

  timer_ = this->create_wall_timer(ms, std::bind(&ImuNodeRcvCsv::Spin, this));
}

~ImuNodeRcvCsv() {
  RCLCPP_INFO(this->get_logger(), "called destructor!");
  imu_.st_ = AdisRcvCsv::State::Ready;
  imu_.md_ = AdisRcvCsv::Mode::Unknown;
  imu_.SendCmd("stop");

  imu_.ClosePort();
}

void InitParam() {
  // Set default value to variables
  device_ = "/dev/ttyACM0";
  frame_id_ = "imu";
  parent_id_ = "base_link";
  rate_ = 100.0;
  
  // Read parameters
  std::string key = "device";
  if (this->get_parameter(key, device_)) {
    RCLCPP_INFO(this->get_logger(), "%s: %s",
      key.c_str(), device_.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), 
      "Could not get param %s. Set default value: %s", key.c_str(), device_.c_str());
  }

  key = "frame_id";
  if (this->get_parameter(key, frame_id_)) {
    RCLCPP_INFO(this->get_logger(), "%s: %s",
      key.c_str(), frame_id_.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(),
      "Could not get param %s. Set default value: %s", key.c_str(), frame_id_.c_str());
  }

  key = "parent_id";
  if (this->get_parameter(key, parent_id_)) {
    RCLCPP_INFO(this->get_logger(), "%s: %s",
      key.c_str(), parent_id_.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(),
      "Could not get param %s. Set default value: %s", key.c_str(), parent_id_.c_str());
  }

  key = "rate"; 
  if (this->get_parameter(key, rate_)) {
    RCLCPP_INFO(this->get_logger(), "%s: %.1f",
      key.c_str(), rate_);
  } else {
    RCLCPP_WARN(this->get_logger(),
      "Could not get param %s. Set default value: %.1f", key.c_str(), rate_);
  }

  cant_rcv_cnt_ = 0;
}

void Diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat) {

  if (cant_rcv_cnt_ >= 1) {
    stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                  "Data cannot be received for more than 1 second.");
  } else {
    stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
  }
}

void InitDevice() {
  std::chrono::seconds sec(1);
  rclcpp::Rate r(sec);
  this->Open();

  while (rclcpp::ok() && !this->IsOpened()) {
    RCLCPP_WARN(this->get_logger(), "Keep trying to open the device in 1 second period...");
    r.sleep();
    this->Open();
  }

  this->Prepare();  
  while (rclcpp::ok() && !this->IsPrepared()){
    RCLCPP_WARN(this->get_logger(), "Keep trying to prepare the device in 1 second period...");
    r.sleep();
    this->Prepare();
  }
}

/**
 * @brief Check if the device is opened
 */
bool IsOpened(void) {
  return (imu_.fd_ >= 0);
}

/**
 * @brief Open IMU device file
 */
bool Open(void) {
  // Open device file
  if (imu_.OpenPort(device_) < 0){
    RCLCPP_ERROR(this->get_logger(), "Failed to open device %s", device_.c_str());
    return false;
  }
  return true;
}

std::string SendCmd(const std::string& cmd, bool is_print = true) {
  std::string ret_str = ""; 
  ret_str = imu_.SendAndRetCmd(cmd);
  if (is_print) {
    RCLCPP_INFO(this->get_logger(), "%s = %s", cmd.c_str(), ret_str.c_str());
  }
  return ret_str;
}

bool CheckFormat() {
  auto ret_str = SendCmd("GET_FORMAT"); 
  if (ret_str == "X_GYRO_HEX,Y_GYRO_HEX,Z_GYRO_HEX,X_ACC_HEX,Y_ACC_HEX,Z_ACC_HEX,CSUM") {
    imu_.md_ = AdisRcvCsv::Mode::Register;
  } else if (ret_str == "YAW[deg],PITCH[deg],ROLL[deg]") {
    imu_.md_ = AdisRcvCsv::Mode::YPR;
  } else {
    imu_.md_ = AdisRcvCsv::Mode::Unknown;
    RCLCPP_WARN(this->get_logger(), "Invalid data format!");
    return false;
  }
  return true;
}

void GetProductId() {
  auto ret_str = SendCmd("GET_PROD_ID");
  updater_->setHardwareID(ret_str);
}

void PrintFirmVersion() {
  SendCmd("GET_VERSION"); 
}

bool CheckStatus() {
  auto ret_str = SendCmd("GET_STATUS"); 
  if (ret_str == "Running") {
    RCLCPP_WARN(this->get_logger(), "Imu state is Running. Send stop command.");
    SendCmd("stop", /* is_print */false);
    return false;
  } else if (ret_str != "Ready") {
    RCLCPP_WARN(this->get_logger(), "Invalid imu state.Waiting Ready state");
    return false;
  }
  return true;
}

bool CheckSensitivity() {
  std::string ret_str = "";
  ret_str = SendCmd("GET_SENSI"); 
  if (ret_str == "") {
    RCLCPP_WARN(this->get_logger(), "Could not get sensitivities!");
    return false;
  } else {
    if (!imu_.SetSensi(ret_str)) {
      RCLCPP_WARN(this->get_logger(), "Insufficient number of sensitivities.");
      return false;
    }
  }
  return true;
}

bool IsPrepared(void) {
  return (imu_.st_ == AdisRcvCsv::State::Running);
}

bool Prepare(void) {
  PrintFirmVersion();
  GetProductId();

  // check imu state
  if (!CheckStatus()) {
    return false;
  }

  imu_.st_ = AdisRcvCsv::State::Ready;

  // check data format
  if (!CheckFormat()) {
    return false;
  }

  // check sensitivity of gyro and acc
  if (imu_.md_ == AdisRcvCsv::Mode::Register) {
    if (!CheckSensitivity()) {
      return false;
    }
  }

  auto ret_str = SendCmd("start", /* is_print */false);
  if (ret_str != "start") {
    RCLCPP_WARN(this->get_logger(), "Send start cmd. but imu was not started.");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Start imu!");
  imu_.st_ = AdisRcvCsv::State::Running;

  return true;
}

void PubImuData() {
  auto data = std::make_shared<sensor_msgs::msg::Imu>();
  
  data->header.frame_id = frame_id_;
  data->header.stamp = this->now();

  // Linear acceleration
  data->linear_acceleration.x = imu_.accl_[0];
  data->linear_acceleration.y = imu_.accl_[1];
  data->linear_acceleration.z = imu_.accl_[2];

  // Angular velocity
  data->angular_velocity.x = imu_.gyro_[0];
  data->angular_velocity.y = imu_.gyro_[1];
  data->angular_velocity.z = imu_.gyro_[2];

  // Orientation (not provided)
  data->orientation.x = 0;
  data->orientation.y = 0;
  data->orientation.z = 0;
  data->orientation.w = 1;

  imu_data_pub_->publish(*data);
}

void BroadcastImuPose() {
  tf2_msgs::msg::TFMessage tf_msg;
  geometry_msgs::msg::TransformStamped tff;

  tff.transform.translation.x = 0;
  tff.transform.translation.y = 0;
  tff.transform.translation.z = 0;

  tf2::Quaternion q;
  q.setRPY(imu_.ypr_[2]*DEG2RAD, imu_.ypr_[1]*DEG2RAD, imu_.ypr_[0]*DEG2RAD);
  tff.transform.rotation.x = q.getX();
  tff.transform.rotation.y = q.getY();
  tff.transform.rotation.z = q.getZ();
  tff.transform.rotation.w = q.getW();

  tff.child_frame_id = frame_id_;
  tff.header.frame_id = parent_id_;
  tff.header.stamp = this->now();

  tf_msg.transforms.push_back(tff);
  tf_br_->publish(tf_msg);
}

void UpdateAndPubRegMode() {
  int res = imu_.UpdateRegMode();
  if (res == IMU_OK) {
    PubImuData();
    cant_rcv_cnt_ = 0;
  } else {
    if (res == IMU_ERR_CANT_RCV_DATA) cant_rcv_cnt_++;

    PrintErrorCode(res);
    RCLCPP_ERROR(this->get_logger(), "Cannot update on reg mode");
  }
}

void UpdateAndPubYprMode() {
  int res = imu_.UpdateYprMode();
  if (res == IMU_OK) {
    BroadcastImuPose();
    cant_rcv_cnt_ = 0;
  } else {
    if (res == IMU_ERR_CANT_RCV_DATA) cant_rcv_cnt_++;

    PrintErrorCode(res);
    RCLCPP_ERROR(this->get_logger(), "Cannot update on ypr mode");
  }
}

void PrintErrorCode(const int& code) {
  switch (code) {
    case IMU_ERR_CANT_RCV_DATA:
      RCLCPP_ERROR(this->get_logger(), "Can not read data from port");
      break;
    case IMU_ERR_COULDNOT_FIND_PACKET:
      RCLCPP_ERROR(this->get_logger(), "Can not find packet");
      break;
    case IMU_ERR_INVALID_DATA:
      RCLCPP_ERROR(this->get_logger(), "Can not invalid data");
      break;
    case IMU_ERR_CHECKSUM:
      RCLCPP_ERROR(this->get_logger(), "Checksum error");
      break;
    default:
      break;
  }
}

void Spin() {
  switch (imu_.md_){
    case AdisRcvCsv::Mode::Register:
      UpdateAndPubRegMode();
      break;
    case AdisRcvCsv::Mode::YPR:
      UpdateAndPubYprMode();
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Unknown imu mode");
      break;
  }
}

};  // end of class


int main(int argc, char **argv) { 
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto imu_node = std::make_shared<ImuNodeRcvCsv>(options);

  rclcpp::spin(imu_node);
  rclcpp::shutdown();
  return 0;
}
