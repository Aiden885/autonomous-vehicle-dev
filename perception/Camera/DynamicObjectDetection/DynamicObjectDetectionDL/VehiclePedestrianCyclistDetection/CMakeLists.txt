cmake_minimum_required(VERSION 2.8.0)
project(smoke_detector)
set(CMAKE_CXX_STANDARD 11)

find_package(PkgConfig REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_library(NVINFER NAMES nvinfer)
find_library(NVINFER_PLUGIN NAMES nvinfer_plugin)
pkg_check_modules(PROTOBUF REQUIRED protobuf)

include_directories(
  ${CUDA_INCLUDE_DIRS}
  ~/workspace/envs/TensorRT-8.6.1.6/include
)

set(MSGDIR /home/mht/code/MsgProto)
add_library(msg_object SHARED ${MSGDIR}/objectListMsg/objectListMsg.pb.cc)
target_link_libraries(msg_object PUBLIC ${PROTOBUF_LIBRARIES})
target_include_directories(msg_object PRIVATE ${PROTOBUF_INCLUDE_DIRS}  PUBLIC ${MSGDIR}/objectListMsg/include)

add_library(smoke_detector SHARED smoke.cc)
target_include_directories(smoke_detector PUBLIC home/mht/code/perceptionv5/perceptionCamera/perceptionCamearaObjectDetection/include)



# add_executable(${PROJECT_NAME}
#   smoke.cc
#   main.cc
# )

target_link_libraries(${PROJECT_NAME}
  # ${NVINFER}
  # ${NVINFER_PLUGIN}
  ~//workspace/envs/TensorRT-8.6.1.6/lib/libnvinfer.so
  ~//workspace/envs/TensorRT-8.6.1.6/lib/libnvinfer_plugin.so
  ${CUDA_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
