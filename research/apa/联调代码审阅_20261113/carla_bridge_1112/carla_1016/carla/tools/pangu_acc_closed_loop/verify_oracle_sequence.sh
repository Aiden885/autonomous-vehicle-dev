#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TMP_DIR="${TMP_DIR:-/tmp/pangu_acc_oracle_sequence}"
SRC="${TMP_DIR}/oracle_sequence_test.cpp"
BIN="${TMP_DIR}/oracle_sequence_test"

mkdir -p "${TMP_DIR}"

cat >"${SRC}" <<'CPP'
#include "AccTargetSpeed.hpp"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool Near(double a, double b, double eps = 1.0e-4) {
  return std::fabs(a - b) <= eps;
}

struct Expected {
  int commandType;
  int systemState;
  int decision;
  bool enable;
  double targetSpeed;
  double timeGap;
  double maxSpeed;
  const char* label;
};

void Check(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "[oracle-test] FAIL: " << message << "\n";
    std::exit(1);
  }
}

}  // namespace

int main() {
  control::AccTargetSpeed acc;
  control::AccTargetSpeed::Input input;
  control::AccTargetSpeed::Output output;

  input.egoSpeed = 2.0;
  input.leadSpeed = 2.0;
  input.leadDistance = 20.0;

  const std::vector<Expected> sequence = {
      {1, 2, 5, false, 0.0, 1.8, 5.0, "E: no-history enable next cycle"},
      {0, 0, 8, true, 5.0, 1.8, 5.0, "hold: enabled keep"},
      {3, 0, 3, true, 5.0, 1.6, 5.0, "T: time-gap down"},
      {4, 0, 4, true, 5.0, 1.8, 5.0, "R: time-gap up"},
      {2, 0, 2, true, 5.0, 1.8, 5.0, "Q: speed up clamped by cap"},
      {1, 0, 1, true, 3.6111, 1.8, 3.6111, "E: speed down"},
      {7, 0, 8, true, 3.6111, 1.8, 3.6111, "C: cancel next cycle"},
      {2, 1, 6, false, 0.0, 1.8, 3.6111, "Q: history resume next cycle"},
      {6, 0, 8, true, 3.6111, 1.8, 3.6111, "S: brake exit next cycle"},
  };

  std::cout << "step,cmd,state,decision,enable,targetSpeed,timeGap,maxSpeed,label\n";
  for (std::size_t i = 0; i < sequence.size(); ++i) {
    const Expected& exp = sequence[i];
    input.commandType = exp.commandType;
    acc.run(input, output);

    std::cout << i << "," << exp.commandType << "," << output.systemState
              << "," << output.decision << "," << (output.enable ? 1 : 0)
              << "," << std::fixed << std::setprecision(4)
              << output.targetSpeed << "," << output.timeGap << ","
              << output.maxSpeed << "," << exp.label << "\n";

    Check(output.systemState == exp.systemState,
          std::string(exp.label) + " systemState");
    Check(output.decision == exp.decision,
          std::string(exp.label) + " decision");
    Check(output.enable == exp.enable,
          std::string(exp.label) + " enable");
    Check(Near(output.targetSpeed, exp.targetSpeed),
          std::string(exp.label) + " targetSpeed");
    Check(Near(output.timeGap, exp.timeGap),
          std::string(exp.label) + " timeGap");
    Check(Near(output.maxSpeed, exp.maxSpeed),
          std::string(exp.label) + " maxSpeed");
  }

  control::AccTargetSpeed accLowSpeed;
  control::AccTargetSpeed::Param paramLowSpeed;
  paramLowSpeed.vMin = 0.5;
  accLowSpeed.setParam(paramLowSpeed);

  input.egoSpeed = 2.0;
  input.leadSpeed = 2.0;
  input.leadDistance = 20.0;
  input.commandType = 1;
  accLowSpeed.run(input, output);
  Check(output.systemState == 2, "low-speed setup start state");
  Check(output.enable == false, "low-speed setup start enable delayed");

  input.commandType = 0;
  accLowSpeed.run(input, output);
  Check(output.systemState == 0, "low-speed setup enabled state");
  Check(output.enable == true, "low-speed setup enabled output");

  input.egoSpeed = 0.1;
  accLowSpeed.run(input, output);
  Check(output.systemState == 3, "low-speed pause state");
  Check(output.enable == false, "low-speed pause output enable");

  input.egoSpeed = 2.0;
  accLowSpeed.run(input, output);
  Check(output.systemState == 0, "low-speed recovery state");
  Check(output.enable == true, "low-speed recovery preserves controlEnabled");

  std::cout << "[oracle-test] PASS\n";
  return 0;
}
CPP

g++ -std=c++17 \
  -I"${ROOT_DIR}/tools/pangu_acc_closed_loop/ACCClosedLoopModule/module_lib" \
  "${SRC}" \
  "${ROOT_DIR}/tools/pangu_acc_closed_loop/ACCClosedLoopModule/module_lib/AccTargetSpeed.cpp" \
  -o "${BIN}"

"${BIN}"
