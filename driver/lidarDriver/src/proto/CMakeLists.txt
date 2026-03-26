cmake_minimum_required(VERSION 3.0)
project(proto)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROTOBUF REQUIRED protobuf)


add_library(lidarMsg SHARED cloud_data.pb.cc)
target_link_libraries(lidarMsg PRIVATE ${PROTOBUF_LIBRARIES})
target_include_directories(lidarMsg PRIVATE ${PROTOBUF_INCLUDE_DIRS} PUBLIC .)
