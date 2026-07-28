#pragma once

#include <string>

namespace pangu::modules {

struct LksZmqState {
  double ego_speed_mps = 0.0;
  double c0 = 0.0;
  double c1 = 0.0;
  double c2 = 0.0;
  double c3 = 0.0;
  double curvature = 0.0;
  bool brake_pressed = false;
  double driver_steer_norm = 0.0;
  bool ego_valid = false;
  bool lane_valid = false;
};

class LksZmqBridge {
public:
  LksZmqBridge();
  ~LksZmqBridge();

  bool Init();
  void Shutdown();
  bool Poll(int timeout_ms);
  bool HasFreshInput() const;
  LksZmqState GetState() const;
  bool PublishControl(double target_speed_mps, double steer_rad, bool enable);

private:
  void* context_ = nullptr;
  void* sub_ = nullptr;
  void* pub_ = nullptr;
  bool initialized_ = false;
  std::string input_endpoint_;
  std::string output_endpoint_;
  unsigned long long sequence_ = 1;
  LksZmqState state_;
  bool fresh_input_ = false;
};

}  // namespace pangu::modules
