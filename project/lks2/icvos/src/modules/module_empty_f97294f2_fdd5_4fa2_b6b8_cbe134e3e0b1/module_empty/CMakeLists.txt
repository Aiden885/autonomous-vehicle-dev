
cmake_minimum_required(VERSION 3.16.1)
##------------------------------------------------------------------##
##------------------------------------------------------------------##
##--以下部分可根据模块组件进行必要修改--###
project(module_empty)   #project里为模块组件名称，与模块目录名一致

set(CORE_LIBS zmq)
include_directories(${THIRD_PATH}/include)

## 在下面可以添加必要的编译选项等
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -O3 -Wno-error=pedantic -pthread -fopenmp")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 -O3 -Werror -Wno-error=pedantic -pthread -fopenmp")
##------------------------------------------------------------------##
##------------------------------------------------------------------##

######################################################################
###!!!以下内容中未加可修改注释的都是模板，不要修改或增加新的内容!!!###
######################################################################
set(MOD_NAME ${PROJECT_NAME})
message("${PROJECT_NAME} CMakeLists Version is 1.5.2")

# generate proto, build and install module_pb
if(NOT COMMON_MESSAGE STREQUAL "no")
    # proto files
    set(proto_dir ${PROJECT_SOURCE_DIR}/proto)
    set(PROTO_CPPS "")
    set(PROTO_HEADERS "")
    set(PROTO_INCLUDE_DIR ${COMMON_MSG_PROTO_DIR}/common ${OS_PROTO_DIR})
    generate_proto(${proto_dir} ${PROTOCPP_DIR} PROTO_CPPS PROTO_HEADERS ${PROTO_INCLUDE_DIR})
    add_library(${PROJECT_NAME}_conf_pb SHARED ${PROTO_CPPS})
    target_link_libraries(${PROJECT_NAME}_conf_pb  ${CONF_PROTO_LIBS} protobuf)
    install(TARGETS ${PROJECT_NAME}_conf_pb
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
    install_depend_libs(${PROJECT_NAME}_conf_pb ${CMAKE_INSTALL_PREFIX}/lib)
endif()

# build and install module_lib
if(EXISTS ${PROJECT_SOURCE_DIR}/module_lib)
    set(MODULELIB_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/module_lib)
    include_directories(${MODULELIB_INCLUDE_DIR})
    FILE(GLOB_RECURSE LIB_SOURCES
        module_lib/*.cc
        module_lib/*.c
        module_lib/*.cpp)
    if (LIB_SOURCES)
        message(STATUS "detect module_lib of ${PROJECT_NAME}, build ${PROJECT_NAME}_core!")
        add_library(${PROJECT_NAME}_core SHARED ${LIB_SOURCES})
        target_link_libraries(${PROJECT_NAME}_core ${CORE_LIBS})
        set(CORE_LIBS ${CORE_LIBS} ${PROJECT_NAME}_core)
        install(TARGETS ${PROJECT_NAME}_core
                LIBRARY DESTINATION lib
                ARCHIVE DESTINATION lib)
        install_depend_libs(${PROJECT_NAME}_core ${CMAKE_INSTALL_PREFIX}/lib)
        if(${BUILD_SHADOW})
            install(TARGETS ${PROJECT_NAME}_core
                    LIBRARY DESTINATION ${SHADOW_DIR}/dependencies/module_lib/lib/${TARGET_PLATFORM}
                    ARCHIVE DESTINATION ${SHADOW_DIR}/dependencies/module_lib/lib/${TARGET_PLATFORM})
        endif()
    elseif(EXISTS ${PROJECT_SOURCE_DIR}/module_lib/lib/${TARGET_PLATFORM}/lib${PROJECT_NAME}_core.so)
      set(CORE_LIBS ${CORE_LIBS} ${PROJECT_SOURCE_DIR}/module_lib/lib/${TARGET_PLATFORM}/lib${PROJECT_NAME}_core.so)
      install(FILES ${PROJECT_SOURCE_DIR}/module_lib/lib/${TARGET_PLATFORM}/lib${PROJECT_NAME}_core.so DESTINATION lib)
    endif()
endif()

# build and install module
set(TARGET_LIBS  ${CORE_LIBS}  ${PROJECT_NAME}_conf_pb ${OS_LIBS}) # 自动配置依赖项自动生成，自动配置工具不具备时，需要手动修改
include_directories(${PROJECT_SOURCE_DIR}/src ${MODULELIB_INCLUDE_DIR})

FILE(GLOB_RECURSE PROJECT_SOURCES
    src/*.c
    src/*.cc
    src/*.cpp)
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*/ZmqBridgeModule\.(cc|cpp|c)$")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*/ZmqBridgeModule\.(cc|cpp|c)$")
add_library(${PROJECT_NAME} SHARED ${PROJECT_SOURCES})
target_link_libraries(${PROJECT_NAME} ${TARGET_LIBS} user_message_msg_pb)
install(TARGETS ${PROJECT_NAME}
                LIBRARY DESTINATION lib
                ARCHIVE DESTINATION lib)
install_depend_libs(${PROJECT_NAME} ${CMAKE_INSTALL_PREFIX}/lib)
FILE(GLOB ZMQ_BRIDGE_SOURCES
    src/ZmqBridgeModule.cc
    src/ZmqBridgeModule.cpp)
if(ZMQ_BRIDGE_SOURCES)
add_library(ZmqBridgeModule SHARED ${ZMQ_BRIDGE_SOURCES})
target_link_libraries(ZmqBridgeModule ${TARGET_LIBS} user_message_msg_pb)
install(TARGETS ZmqBridgeModule
                LIBRARY DESTINATION lib
                ARCHIVE DESTINATION lib)
install_depend_libs(ZmqBridgeModule ${CMAKE_INSTALL_PREFIX}/lib)
endif()

# build unit test
if(UTEST)
    set(TARGET_LIBS ${TARGET_LIBS} ${PROJECT_NAME} protobuf_tools)
    set(UTEST_BIN_PATH ${TEST_OUTPUT_PATH}/bin/${MOD_NAME}/)
    add_subdirectory(test)
    file(COPY conf DESTINATION ${UTEST_BIN_PATH})
endif(UTEST)

# install conf
install(DIRECTORY conf/ DESTINATION conf/node_module/${PROJECT_NAME})
