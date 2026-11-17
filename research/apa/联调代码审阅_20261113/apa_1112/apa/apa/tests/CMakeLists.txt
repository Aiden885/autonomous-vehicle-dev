cmake_minimum_required(VERSION 3.16)
project(apa_regression C CXX)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/ApaVehicleProfile.cmake")
apa_generate_vehicle_profile(APA_GENERATED_PROFILE_DIR)
include_directories(BEFORE "${APA_GENERATED_PROFILE_DIR}")
set(APA_MODULE_LIB "${CMAKE_CURRENT_LIST_DIR}/../module_lib")
set(APA_DIR "${APA_MODULE_LIB}/apa_algorithm/apa")
file(GLOB APA_C_SOURCES CONFIGURE_DEPENDS "${APA_DIR}/*.c")
add_library(apa_runtime STATIC ${APA_C_SOURCES} "${APA_MODULE_LIB}/apa_parking_search.cpp")
target_include_directories(apa_runtime PUBLIC "${APA_DIR}")
target_compile_options(apa_runtime PRIVATE -Wall -Wextra -Werror)
target_link_libraries(apa_runtime PUBLIC m)
add_subdirectory("${APA_MODULE_LIB}/apa_algorithm/XXX" funcmodule)

enable_testing()
function(apa_add_regression name source)
  add_executable(apa_${name} ${source})
  target_compile_options(apa_${name} PRIVATE -Wall -Wextra -Werror -UNDEBUG)
  target_link_libraries(apa_${name} PRIVATE apa_runtime)
  add_test(NAME ${name} COMMAND apa_${name})
endfunction()
apa_add_regression(tracking_regression tracking_regression.c)
apa_add_regression(actuator_regression actuator_regression.c)
apa_add_regression(slot_guidance_regression slot_guidance_regression.c)
apa_add_regression(parking_search_direction_regression parking_search_direction_regression.c)

option(APA_BUILD_INPUT_CONTRACT_TESTS "Build input contract checks against generated APA Proto" OFF)
if(APA_BUILD_INPUT_CONTRACT_TESTS)
  get_filename_component(APA_PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
  set(APA_PROTO_BUILD_DIR "${APA_PROJECT_ROOT}/build_out" CACHE PATH
      "Pangu build outputs containing protocpp/apa_config.pb.h and lib/libapa_conf_pb.so")
  if(NOT EXISTS "${APA_PROTO_BUILD_DIR}/protocpp/apa_config.pb.h" OR
      NOT EXISTS "${APA_PROTO_BUILD_DIR}/lib/libapa_conf_pb.so")
    message(FATAL_ERROR "Build the Pangu apa_conf_pb target first, or set APA_PROTO_BUILD_DIR")
  endif()
  add_executable(apa_input_contract_regression input_contract_regression.cpp
      online_safety_regression.cpp ../module_lib/apa_input_contract.cpp
      ../module_lib/apa_online_safety.cpp ../module_lib/apa_control_limits.cpp
      ../module_lib/apa_motion_prediction.cpp)
  target_include_directories(apa_input_contract_regression PRIVATE
      ../module_lib "${APA_PROTO_BUILD_DIR}/protocpp"
      "${APA_PROJECT_ROOT}/dependencies/common/protocpp"
      "${APA_PROJECT_ROOT}/dependencies/common/os_adapter/icvos/X86/sdk/include"
      "${APA_PROJECT_ROOT}/dependencies/thirdparty/X86/include")
  target_include_directories(apa_input_contract_regression SYSTEM PRIVATE
      "${APA_DIR}" ../module_lib/apa_algorithm/XXX/include)
  target_link_directories(apa_input_contract_regression PRIVATE
      "${APA_PROTO_BUILD_DIR}/lib" "${APA_PROJECT_ROOT}/dependencies/thirdparty/X86/lib"
      "${APA_PROJECT_ROOT}/dependencies/common/os_adapter/icvos/X86/sdk/lib")
  target_compile_options(apa_input_contract_regression PRIVATE -Wall -Wextra -Werror -UNDEBUG)
  target_link_libraries(apa_input_contract_regression PRIVATE apa_runtime apa_conf_pb os_proto_pb protobuf m)
  add_test(NAME input_contract_regression COMMAND apa_input_contract_regression)
endif()
