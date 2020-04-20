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
#pragma once


#include <vector>
#include <string>

#include <termios.h>


#ifndef RING_BUF_SIZE
#define RING_BUF_SIZE (2000)
#endif

#ifndef GRAVITY
#define GRAVITY (9.80655)
#endif

#ifndef DEG2RAD
#define DEG2RAD(deg) (deg*0.01745329251)
#endif

#ifndef RAD2DEG
#define RAD2DEG(rad) (rad*57.2957795131)
#endif

#ifndef IMU_OK
#define IMU_OK (0)
#endif
#ifndef IMU_ERR_CANT_RCV_DATA
#define IMU_ERR_CANT_RCV_DATA (1)
#endif
#ifndef IMU_ERR_COULDNOT_FIND_PACKET
#define IMU_ERR_COULDNOT_FIND_PACKET (2)
#endif
#ifndef IMU_ERR_INVALID_DATA
#define IMU_ERR_INVALID_DATA (3)
#endif
#ifndef IMU_ERR_CHECKSUM
#define IMU_ERR_CHECKSUM (4)
#endif

class AdisRcvCsv {
public:
  AdisRcvCsv();
  ~AdisRcvCsv();

  enum class State {
    Ready,
    Running,
    Unknown
  };

  enum class Mode {
    YPR,
    Register,
    Unknown
  };

  //! File descripter for USB-ISS
  int fd_;
  //! Saved terminal config
  struct termios defaults_;

  // Gyro sensor(x, y, z)
  double gyro_[3];
  // Acceleration sensor(x, y, z)
  double accl_[3];
  // estimated imu pose(Yaw, Pitch, Roll)[deg]
  double ypr_[3];

  double gyro_sensi_;
  double acc_sensi_;
  char ring_buf_[RING_BUF_SIZE];
  int ring_pointer_;

  int UpdateRegMode(void);
  int UpdateYprMode(void);

  int OpenPort(const std::string& device);
  void ClosePort();
  int ReadSerial();
  int WriteSerial(const std::string& cmd);
  
  bool SendCmd(const std::string& cmd);
  std::string SendAndRetCmd(const std::string& cmd);

  int CalNextPointer(const int& src);
  int CalPrePointer(const int& src);
  std::string FindLastData();
  std::string FindCmdReturnRow(const std::string& cmd);
  std::vector<std::string> Split(const std::string& str, const char& delm);
  bool SetSensi(const std::string& sensi_str);
  int MakeCsum(const std::vector<int>& data);
//  void clearBuf();

  State st_;
  Mode md_;
};
