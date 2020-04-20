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
#include <adi_driver2/adis_rcv_csv.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>

#include <memory.h>
#include <sstream>

using namespace std;
//using namespace std::chrono_literals;

AdisRcvCsv::AdisRcvCsv() {
  fd_ = -1;
  ring_pointer_ = 0;
  st_ = State::Unknown;
  md_ = Mode::Unknown;
}

AdisRcvCsv::~AdisRcvCsv() {}

/**
 * @brief Open device
 * @param device Device file name (/dev/ttyACM*)
 * @retval 0 Success
 * @retval -1 Failure
 */
int AdisRcvCsv::OpenPort(const std::string& device) {
  fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0){
    perror("openPort");
    return -1;
  }

  if (tcgetattr(fd_, &defaults_) < 0) {
    perror("openPort");
    return -1;
  }
  struct termios config;
  cfmakeraw(&config);
  config.c_cc[VMIN] = 0;    // non block
  config.c_cc[VTIME] = 10;  // 1 second read timeout

  if (tcsetattr(fd_, TCSANOW, &config) < 0) {
    perror("openPort");
    return -1;
  }
  return 0;
}

/**
 * @brief Close device
 */
void AdisRcvCsv::ClosePort() {
  if (tcsetattr(fd_, TCSANOW, &defaults_) < 0) {
    perror("closePort");
  }
  close(fd_);
}


int AdisRcvCsv::ReadSerial() {
  int rcv_cnt = -1;
  constexpr int read_buf_size = 1000;
  char buf[read_buf_size];

  rcv_cnt = read(fd_, buf, read_buf_size);

  // write ring buffer
  for (int i = 0; i < rcv_cnt; i++)  {
    ring_pointer_++;
    ring_pointer_ = ring_pointer_ % RING_BUF_SIZE;
    ring_buf_[ring_pointer_] = buf[i];
  }

 #if 0
//  printf("rcv_cnt: %d\n", rcv_cnt);
  ring_buf_[RING_BUF_SIZE-1] = '\0';
  printf("%s\n\n", ring_buf_);
 #endif
 return rcv_cnt;
}

int AdisRcvCsv::WriteSerial(const std::string& cmd) {
  int write_cnt = -1;
  write_cnt = write(fd_, cmd.c_str(), cmd.size());

  if (write_cnt < 0) {
    fprintf(stdout, "Could not write to serial port %d\n", write_cnt);
  }
 return write_cnt;
}

std::string AdisRcvCsv::SendAndRetCmd(const std::string& cmd) {
  std::string ret_str = "";
  std::string tmp_str = "";
  std::string err_str = "";

  SendCmd(cmd);
  ReadSerial(); // store data to ringbuf
//  std::string tmp_str = findLastData();
  tmp_str = FindCmdReturnRow(cmd);

  err_str = FindCmdReturnRow("ERROR");
  if (err_str != "") {
    memset(ring_buf_, '\0', RING_BUF_SIZE); // clear buffer
    return err_str;
  }

  auto splited = Split(tmp_str,  ',');

  if (splited.size() >= 2) {
    // is include cmd string?
    if (splited[0].find(cmd) != std::string::npos) {
      for (size_t i = 1; i < splited.size(); i++) {
        ret_str += splited[i] + ",";
      }
      ret_str.erase(ret_str.end()-1); // erase last comma
    }
  } else if (splited.size() == 1){
    if (splited[0].find(cmd) != std::string::npos) {
      ret_str = splited[0];
    }
  }
  return ret_str;
}

bool AdisRcvCsv::SendCmd(const std::string& cmd) {
  auto success = WriteSerial(cmd + "\r\n");
  usleep(100000); // 100ms
  return success;
}

std::string AdisRcvCsv::FindCmdReturnRow(const std::string& cmd) {
  int wp = ring_pointer_;
  std::string ret_str = "";

  int ii = 0;
  for (; ii < RING_BUF_SIZE; ii++) {
    if (ring_buf_[wp] == cmd[0]) {
      bool find_flg = true;
      int wp2 = CalNextPointer(wp);
      for (size_t j = 1; j < cmd.size(); j++) {
        if (cmd[j] != ring_buf_[wp2]) {
          find_flg = false;
          break;  // inner for
        }
        wp2 = CalNextPointer(wp2);
      }
      if (find_flg){
        break;  // outer for
      }
    }    
    wp = CalPrePointer(wp);
  }
  if (ii == RING_BUF_SIZE) {
    if (cmd != "ERROR") {
      printf("Can not find cmd from ring buffer\n");
    }
    return ret_str;
  }

  for (size_t i = 0; i < RING_BUF_SIZE; i++) {
    ret_str += ring_buf_[wp];
    wp = CalNextPointer(wp);
    if (ring_buf_[wp] == '\r') {
      break;
    }
  }
  return ret_str;
}

std::string AdisRcvCsv::FindLastData() {
  int wp = ring_pointer_;
  int n_cnt = 0;
  int last_index = -1;
  int pre_last_index = -1;

  // search two \r\n from last
  for (size_t i = 0; i < RING_BUF_SIZE; i++) {
    if (ring_buf_[wp] == '\r' 
      && ring_buf_[CalNextPointer(wp)] == '\n') {
      n_cnt++;
      if (n_cnt == 2) {
        pre_last_index = wp;
        break;
      } else {
        last_index = wp;
      }
    }
    wp = CalPrePointer(wp);
  }

  // +1 mean skip \r
  int index = CalNextPointer(pre_last_index+1);

  std::string ret_string;
  for (size_t i = 0; i < RING_BUF_SIZE; i++) {
    if (index == last_index) {
      break;
    } else {
      ret_string += ring_buf_[index];
    }
    index++;
    index = index % RING_BUF_SIZE;
  }
  return ret_string;
}


/**
 * @brief update gyro and accel in high-precision read
 */
int AdisRcvCsv::UpdateRegMode(void) {
  if (ReadSerial() <= 0) {
    return IMU_ERR_CANT_RCV_DATA;
  }
  
  std::string row = FindLastData();
  if (row.size() == 0) {
    return IMU_ERR_INVALID_DATA;
  }

  auto splited_data = Split(row, ',');
  if (splited_data.size() != 7) {
    return IMU_ERR_INVALID_DATA;
  }

  std::vector<int> num_data(6, 0);
  int csum = 300;
  try {
    for (size_t i = 0; i < num_data.size(); i++) {
      num_data[i] = (int)std::stol(splited_data[i], nullptr, 16);
    }
    csum = (int)std::stol(splited_data.back(), nullptr, 16);
  } catch (std::invalid_argument e) {
    printf("%s\n", e.what());
  }

  if (MakeCsum(num_data) != csum) {
    printf("Invalid checksum!\n");
    return 1;
  }

  for (size_t i = 0; i < 3; i++){
    gyro_[i] = DEG2RAD((double)num_data[i])    / gyro_sensi_;
    accl_[i] = (double)num_data[i+3] * GRAVITY / acc_sensi_;
  } 
  return IMU_OK;
}

/**
 * @brief update YawPitchRoall in high-precision read
 */
int AdisRcvCsv::UpdateYprMode(void) {
  if (ReadSerial() <= 0) {
    return IMU_ERR_CANT_RCV_DATA;
  }
  
  std::string row = FindLastData();
  if (row.size() == 0) {
    return IMU_ERR_COULDNOT_FIND_PACKET;
  }

  auto splited_data = Split(row, ',');
  if (splited_data.size() != 3) {
    return IMU_ERR_INVALID_DATA;
  }

  try {
    for (size_t i = 0; i < 3; i++) {
      ypr_[i] = std::stof(splited_data[i]);
    }
  } catch (std::invalid_argument e) {
    printf("%s\n", e.what());
  }
  return IMU_OK;
}

int AdisRcvCsv::MakeCsum(const std::vector<int>& array) {
  int sum = 0;
  for (size_t i = 0; i < array.size(); i++) {
    sum += (array[i]>>24) & 0xff;
    sum += (array[i]>>16) & 0xff;
    sum += (array[i]>>8)  & 0xff;
    sum += (array[i])     & 0xff;
  }
  return (sum & 0xff); 
}

int AdisRcvCsv::CalNextPointer(const int& src) {
  return ((src+1) % RING_BUF_SIZE);
}

int AdisRcvCsv::CalPrePointer(const int& src) {
  return (src+RING_BUF_SIZE-1) % RING_BUF_SIZE;
}

std::vector<std::string> AdisRcvCsv::Split(const std::string& str, const char& delm) {
  std::vector<std::string> ret;
  std::istringstream stream(str);
  std::string tmp;

  while (getline(stream, tmp, delm)) ret.push_back(tmp);
  return ret;
}

bool AdisRcvCsv::SetSensi(const std::string& sensi_str) {
  auto splited = Split(sensi_str, ',');
  if (splited.size() != 2) {
    return false;
  }
  gyro_sensi_ = std::stod(splited[0]);
  acc_sensi_ = std::stod(splited[1]);
//  printf("%f, %f\n", gyro_sensi_, acc_sensi_);
  return true; 
}