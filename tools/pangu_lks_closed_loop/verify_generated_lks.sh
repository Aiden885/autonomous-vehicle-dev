#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SOURCE_DIR="${ROOT_DIR}/project/lks2/icvos/src/temp_codegen_output"
BUILD_DIR="${LKS_CODEGEN_BUILD_DIR:-/tmp/lks2_codegen_build}"
TEST_CPP="${BUILD_DIR}/lks_smoke.cpp"

cmake -S "${SOURCE_DIR}" -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}" -j"$(nproc)"

cat >"${TEST_CPP}" <<'EOF'
#include <cmath>
#include <iostream>
#include "run_6d9ae1e1_3798_4640_91c6_8217a06e02aa.hpp"

int main() {
  control::run_6d9ae1e1_3798_4640_91c6_8217a06e02aa controller;
  control::run_6d9ae1e1_3798_4640_91c6_8217a06e02aaTraits::Input input{};
  control::run_6d9ae1e1_3798_4640_91c6_8217a06e02aaTraits::Output output{};
  input.egoV = 5.0;
  input.c0 = 0.8;
  controller.run(input, output);
  std::cout << "preview=" << output.previewDistance
            << " weighted_error=" << output.weightedError
            << " steer_rad=" << output.lksSteerRad
            << " enabled=" << output.controlEnabled << std::endl;
  const bool normal_ok = std::abs(output.previewDistance - 7.5) < 1e-6 &&
                         std::abs(output.weightedError - 0.8) < 1e-6 &&
                         std::abs(output.lksSteerRad - 0.0384) < 1e-6 &&
                         output.controlEnabled == 1.0;

  input.driverSteerNorm = 0.3;
  controller.run(input, output);
  const bool steer_takeover_ok = output.controlEnabled == 0.0 &&
                                 std::abs(output.lksSteerRad) < 1e-9;
  std::cout << "steer_takeover enabled=" << output.controlEnabled
            << " steer_rad=" << output.lksSteerRad << std::endl;

  input.driverSteerNorm = 0.0;
  input.brakePressed = 1.0;
  controller.run(input, output);
  const bool brake_ok = output.controlEnabled == 0.0 &&
                        std::abs(output.lksSteerRad) < 1e-9;
  std::cout << "brake enabled=" << output.controlEnabled
            << " steer_rad=" << output.lksSteerRad << std::endl;

  input.brakePressed = 0.0;
  input.egoV = 0.5;
  controller.run(input, output);
  const bool low_speed_ok = output.controlEnabled == 0.0 &&
                            std::abs(output.lksSteerRad) < 1e-9;
  std::cout << "low_speed enabled=" << output.controlEnabled
            << " steer_rad=" << output.lksSteerRad << std::endl;
  return normal_ok && steer_takeover_ok && brake_ok && low_speed_ok ? 0 : 1;
}
EOF

g++ -std=c++2a -I"${SOURCE_DIR}" "${TEST_CPP}" \
  -L"${BUILD_DIR}/lib" -lMainInclude_shared -Wl,-rpath,"${BUILD_DIR}/lib" \
  -o "${BUILD_DIR}/lks_smoke"
"${BUILD_DIR}/lks_smoke"
