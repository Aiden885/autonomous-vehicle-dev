if(TARGET apa_core)
  # Production collects these sources into apa_core in the parent directory.
  return()
endif()
if(NOT TARGET apa_runtime)
  message(FATAL_ERROR "Build FuncModule checks through modules/apa/apa/tests")
endif()
if(NOT APA_GENERATED_PROFILE_DIR)
  include("${CMAKE_CURRENT_LIST_DIR}/../../../cmake/ApaVehicleProfile.cmake")
  apa_generate_vehicle_profile(APA_GENERATED_PROFILE_DIR)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(apa_cpp_funcmodule STATIC
  ../../apa_native_planning.cpp
  src/GlobalContext.cpp
  src/ApaParkingPathPlanning.cpp
  src/ApaParkingRouteSelection.cpp
  src/ApaParkingSlotPathPlanning.cpp
  src/ApaParkingPathPlanningZeroOutput.cpp
  src/ApaParkingOutPathPlanning.cpp
  src/ApaParkingOutTargetConnector.cpp
  src/ApaVtParkingPathPlanning.cpp
  src/ApaHtParkingPathPlanning.cpp
  src/ApaDthEdVtParkingPathPlanning.cpp
  src/ApaVhParkingPathPlanning.cpp
  src/ApaParkingPathSafetyGate.cpp
  src/ApaRecoveryDecision.cpp
  src/ApaRecoveryObservation.cpp
  src/ApaVirtualRecoveryPrefixPlanning.cpp
)

target_include_directories(apa_cpp_funcmodule PUBLIC
  "${APA_GENERATED_PROFILE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/../.."
  "${CMAKE_CURRENT_SOURCE_DIR}/../apa"
)

target_link_libraries(apa_cpp_funcmodule PUBLIC apa_runtime m)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(apa_cpp_funcmodule PRIVATE
    -Wall
    -Wextra
    -Werror
  )
endif()

if(APA_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(apa_cpp_funcmodule PRIVATE
    -fsanitize=address,undefined
    -fno-omit-frame-pointer
  )
  target_link_options(apa_cpp_funcmodule INTERFACE -fsanitize=address,undefined)
endif()
