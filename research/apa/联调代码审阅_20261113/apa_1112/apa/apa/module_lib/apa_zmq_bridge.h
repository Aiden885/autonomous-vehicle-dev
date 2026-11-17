#pragma once

#include <string>
#include <cstdint>
#include <mutex>
#include <vector>

#include "apa_config.pb.h"
#include "apa_input_contract.h"

namespace pangu::modules {

class ApaZmqInputBridge {
public:
  ApaZmqInputBridge();
  ~ApaZmqInputBridge();

  bool Init();
  void Shutdown();
  bool Poll(int timeout_ms);
  bool HasFreshInput() const;
  ApaInput GetInput() const;

private:
  void* context_ = nullptr;
  void* sub_ = nullptr;
  bool initialized_ = false;
  std::string input_endpoint_;
  ApaInputContract input_contract_;
  bool fresh_input_ = false;
};

class ApaZmqOutputBridge {
public:
  ApaZmqOutputBridge();
  ~ApaZmqOutputBridge();

  bool Init();
  void Shutdown();
  bool Publish(const ApaOutput& output);

private:
  bool PublishJson(const char* topic, const std::string& text);

  void* context_ = nullptr;
  void* pub_ = nullptr;
  bool initialized_ = false;
  std::string output_endpoint_;
  std::string source_;
  unsigned long long sequence_ = 1;
  std::mutex publish_mutex_;
};

}
