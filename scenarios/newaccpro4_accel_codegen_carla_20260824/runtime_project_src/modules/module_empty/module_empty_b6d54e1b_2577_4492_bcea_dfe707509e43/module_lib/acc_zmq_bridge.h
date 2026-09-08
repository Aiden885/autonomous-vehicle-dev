#pragma once

#include <string>

namespace pangu::modules {

struct AccZmqState {
  double ego_speed_mps = 0.0;
  double lead_speed_mps = 0.0;
  double lead_distance_m = 1000000.0;
  double relative_speed_mps = 0.0;
  double ttc_sec = 1000000.0;
  bool ego_valid = false;
  bool lead_valid = false;
  int command_type = 0;
  bool command_valid = false;
};

class AccZmqInputBridge {
public:
  AccZmqInputBridge();
  ~AccZmqInputBridge();

  bool Init();
  void Shutdown();
  bool Poll(int timeout_ms);
  bool HasFreshInput() const;
  AccZmqState GetState() const;

private:
  void* context_ = nullptr;
  void* sub_ = nullptr;
  bool initialized_ = false;
  std::string input_endpoint_;
  AccZmqState state_;
  bool fresh_input_ = false;
};

class AccZmqOutputBridge {
public:
  AccZmqOutputBridge();
  ~AccZmqOutputBridge();

  bool Init();
  void Shutdown();
  bool PublishAccelerationControl(double acceleration_command_mps2,
                                  int enable);

private:
  void* context_ = nullptr;
  void* pub_ = nullptr;
  bool initialized_ = false;
  std::string output_endpoint_;
  std::string source_;
  std::string frame_id_;
  std::string ego_role_name_;
  std::string coordinate_frame_;
  unsigned long long sequence_ = 1;
};

}  // namespace pangu::modules
