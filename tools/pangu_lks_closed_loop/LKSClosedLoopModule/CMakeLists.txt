cmake_minimum_required(VERSION 3.16.1)
project(LKSModule)

set(CORE_LIBS zmq)
include_directories(${THIRD_PATH}/include)

if(NOT DEFINED LKS_GENERATED_DIR)
  message(FATAL_ERROR "LKS_GENERATED_DIR must point to lks2/temp_codegen_output")
endif()
include_directories(${LKS_GENERATED_DIR})

set(proto_dir ${PROJECT_SOURCE_DIR}/proto)
set(PROTO_CPPS "")
set(PROTO_HEADERS "")
set(PROTO_INCLUDE_DIR ${COMMON_MSG_PROTO_DIR}/common ${OS_PROTO_DIR})
generate_proto(${proto_dir} ${PROTOCPP_DIR} PROTO_CPPS PROTO_HEADERS ${PROTO_INCLUDE_DIR})
add_library(${PROJECT_NAME}_conf_pb SHARED ${PROTO_CPPS})
target_link_libraries(${PROJECT_NAME}_conf_pb ${CONF_PROTO_LIBS} protobuf)
install(TARGETS ${PROJECT_NAME}_conf_pb LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install_depend_libs(${PROJECT_NAME}_conf_pb ${CMAKE_INSTALL_PREFIX}/lib)

file(GLOB_RECURSE LIB_SOURCES module_lib/*.cpp)
file(GLOB_RECURSE GENERATED_SOURCES
  ${LKS_GENERATED_DIR}/GlobalContext/*.cpp
  ${LKS_GENERATED_DIR}/composite_block_empty_*/*.cpp
  ${LKS_GENERATED_DIR}/run_*/*.cpp)
add_library(${PROJECT_NAME}_core SHARED ${LIB_SOURCES} ${GENERATED_SOURCES})
target_link_libraries(${PROJECT_NAME}_core ${CORE_LIBS} m)
install(TARGETS ${PROJECT_NAME}_core LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install_depend_libs(${PROJECT_NAME}_core ${CMAKE_INSTALL_PREFIX}/lib)

set(TARGET_LIBS ${CORE_LIBS} ${PROJECT_NAME}_core ${PROJECT_NAME}_conf_pb ${OS_LIBS})
include_directories(${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/module_lib)

add_library(${PROJECT_NAME} SHARED src/LKSModule.cc)
target_link_libraries(${PROJECT_NAME} ${TARGET_LIBS} user_message_msg_pb)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install_depend_libs(${PROJECT_NAME} ${CMAKE_INSTALL_PREFIX}/lib)

add_library(ZmqBridgeModule SHARED src/ZmqBridgeModule.cc)
target_link_libraries(ZmqBridgeModule ${TARGET_LIBS} user_message_msg_pb)
install(TARGETS ZmqBridgeModule LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install_depend_libs(ZmqBridgeModule ${CMAKE_INSTALL_PREFIX}/lib)

install(DIRECTORY conf/ DESTINATION conf/node_module/${PROJECT_NAME})
