cmake_minimum_required(VERSION 3.16.1)
project(pangu)
add_definitions(-w)
add_definitions(-DSYSTEM_NAME=\"${PROJECT_NAME}\")
if(ENABLE_DEBUG)
    set (CMAKE_BUILD_TYPE Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -w -g -fpermissive -fPIC")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O2 -fpermissive")
    add_definitions(-DENABLE_DEBUG)
else()
    set (CMAKE_BUILD_TYPE Release)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -W -Werror=return-type -O2 -fpermissive -fPIC")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -fpermissive")
endif(ENABLE_DEBUG)

if (NOT ARCH MATCHES "aarch64")
    set(ARCH "x86")
    set(IS_X86 1)
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -faligned-new")

    if(UTEST)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
    endif()
    # option(INSTALL_THIRDPARTY_LIB "install would package thirdparty libraries or not" OFF)
    option(INSTALL_THIRDPARTY_LIB "install would package thirdparty libraries or not" ON)
else()
    set(ARCH "aarch64")
    set(IS_X86 0)    
    option(INSTALL_THIRDPARTY_LIB "install would package thirdparty libraries or not" ON)
endif() 

option(BUILD_SHADOW "build shadow for gaasd" OFF)

set(WITH_ROS_PATH no_ros)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color")

if(${IS_X86})
    set(USE_ROS off)
    set(USE_ROS2 on)
    add_definitions(-DUSE_ROS2)
    set(WITH_ROS_PATH ros2)
else()
    set(USE_ROS2 off)
    set(USE_ROS off)
endif()

# -----------------------------设置OS-----------------------------
# 设置OS
set(RUNTIME_OS ICVOS CACHE STRING "What Should I RUN IN? (cmake -DRUNTIME_OS=[ICVOS] ..)")
message("RUNTIME_OS: ${RUNTIME_OS}")


# -----------------------------设置平台，架构相关宏START-----------------------------
# 设置不同运行平台， 不同架构
set(BUILD_TARGET PC CACHE STRING "What Should I Build? (cmake -DBUILD_TARGET=[PC, MDC510, Xavier] ..)")
set(HOST_PLATFORM x86_64 CACHE STRING "What Should I Build? (cmake -DHOST_PLATFORM=[x86_64, aarch64] ..)")

# ## PC:X86_64 (tensorrt)
# ## MDC510: x86_64(j5_x86), aarch64(j5)
# ## Xavier: aarch64 (tensorrt)
# Compile settings
set(ENABLE_TENSORRT ON)
set(ENABLE_MDC_510 OFF)
set(ENABLE_MDC_610 OFF)
set(ENABLE_X86 OFF)
set(ENABLE_J6 OFF)
set(ENABLE_C1200 OFF)
if (${BUILD_TARGET} STREQUAL MDC_510)
    message("[BUILD] Building MDC 510 version")
    set(ENABLE_TENSORRT OFF)
    set(ENABLE_MDC_510 ON)
elseif(${BUILD_TARGET} STREQUAL MDC_610)
    message("[BUILD] Building MDC 610 version")
    set(ENABLE_TENSORRT OFF)
    set(ENABLE_MDC_610 ON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics")
elseif(${BUILD_TARGET} STREQUAL J6)
    message("[BUILD] Building J6 version")
    set(ENABLE_TENSORRT OFF)
    set(ENABLE_J6 ON)
elseif(${BUILD_TARGET} STREQUAL X86)
    message("[BUILD] Building X86 version")
    set(ENABLE_TENSORRT OFF)
    set(ENABLE_X86 ON)
elseif(${BUILD_TARGET} STREQUAL C1200)
    message("[BUILD] Building C1200 version")
    set(ENABLE_TENSORRT OFF)
    set(ENABLE_C1200 ON)
endif()

add_definitions(-DICVOS)

# -----------------------------设置TENSORRT的相关依赖-----------------------
if(${ENABLE_TENSORRT})
    add_definitions(-DENABLE_TENSORRT=1)
elseif(${ENABLE_MDC_510})
    add_definitions(-DENABLE_MDC_510=1)
elseif(${ENABLE_MDC_610})
    add_definitions(-DENABLE_MDC_610=1)
elseif(${ENABLE_J6})
    add_definitions(-DENABLE_J6=1)
elseif(${ENABLE_C1200})
    add_definitions(-DENABLE_C1200=1)
endif()

add_definitions(-DPROTOCONFIG)

add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

set(TARGET_PLATFORM X86)

if(${ENABLE_MDC_510})
    set(TARGET_PLATFORM MDC510)
elseif(${ENABLE_MDC_610})
    set(TARGET_PLATFORM MDC610)
elseif(${ENABLE_J6})
    set(TARGET_PLATFORM J6)
elseif(${ENABLE_C1200})
    set(TARGET_PLATFORM C1200)    
elseif(ARCH MATCHES "aarch64")
    set(TARGET_PLATFORM AARCH64)
endif()


if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------modules' definitions-------------------------
# ---------------------------fusion tracker---------------------------
option(MSF_VIZ "option for MSF_VIZ" OFF)
if (MSF_VIZ)
    add_definitions(-DMSF_VIZ)
endif()
option(MSF_DEBUG "option for MSF_DEBUG" OFF)
if (MSF_DEBUG)
    add_definitions(-DMSF_DEBUG)
endif()
# --------------------------fusion tracker end------------------------

# ---------------------modules' definitions end-----------------------

set(BUILD_OUT_DIR ${CMAKE_SOURCE_DIR}/build_out)
set(PROTOCPP_DIR ${CMAKE_SOURCE_DIR}/build_out/protocpp)
set(LIBRARY_OUTPUT_PATH ${BUILD_OUT_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${BUILD_OUT_DIR}/bin)
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
set(DEPS_DIR ${CMAKE_SOURCE_DIR}/dependencies)
set(COMMON_DIR ${DEPS_DIR}/common)
set(TEST_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/test)
set(TEST_LIB_COPY_DIR ${TEST_OUTPUT_PATH}/lib)
set(SHADOW_DIR ${CMAKE_SOURCE_DIR}/shadow/cbdes_int)

if(NOT UTEST)
    set(UTEST OFF)
endif()
include_directories(${PROTOCPP_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${DEPS_DIR})
link_directories(${BUILD_OUT_DIR}/lib)

set(DEPLOAD on)
include(${DEPS_DIR}/deps.cmake)
set(TARGET_PLATFORM ${RUNTIME_OS}_${TARGET_PLATFORM})

set(ALL_MODULES on)
if(DEFINED OPEN_WHITE_LIST AND OPEN_WHITE_LIST)
    set(ALL_MODULES off)
else()
    set(ALL_MODULES on)
endif()


include(${COMMON_DIR}/common_pangu.cmake)

add_subdirectory(dependencies/user_message)
if(EXISTS ${ALGO_MODULES_DIR}/proto)
  add_subdirectory(${ALGO_MODULES_DIR}/proto)
endif()

set(MDC_LIBS)

if(${ENABLE_MDC_510})
        set(MDC_LIBS adb ascendcl camera mdc_mbuf log opencv_world acl_tdt_queue acl_dvpp_mpi)
elseif(${ENABLE_MDC_610})
        set(MDC_LIBS adb ascendcl log opencv_world acl_tdt_queue acl_dvpp_mpi)
elseif(${ENABLE_J6})
        set(MDC_LIBS )
elseif(${ENABLE_C1200})
        set(MDC_LIBS )
endif()

add_subdirectory(modules)



install(FILES ${PROJECT_SOURCE_DIR}/version.hpp DESTINATION .)
install(FILES ${PROJECT_SOURCE_DIR}/script/ps.sh DESTINATION .)
install(FILES ${PROJECT_SOURCE_DIR}/script/stop_all.sh DESTINATION .)
install(FILES ${PROJECT_SOURCE_DIR}/script/stop.sh DESTINATION .)
install(FILES ${PROJECT_SOURCE_DIR}/script/run_all.sh DESTINATION .)
install(FILES ${PROJECT_SOURCE_DIR}/script/run.sh DESTINATION .)
install(FILES ${PROJECT_SOURCE_DIR}/script/color.bash DESTINATION .)
install(FILES ${PROJECT_SOURCE_DIR}/script/setup.bash DESTINATION .)
install(FILES ${PROJECT_SOURCE_DIR}/script/pangu_completion.sh DESTINATION .)
install(FILES ${PROJECT_SOURCE_DIR}/script/load_configs.sh DESTINATION .)
#install(FILES ${PROJECT_SOURCE_DIR}/script/pytime.py DESTINATION .)
install(FILES ${PROJECT_SOURCE_DIR}/script/cross_host.sh DESTINATION .)
install(FILES ${PROJECT_SOURCE_DIR}/script/launch.sh DESTINATION .)
install(FILES ${PROJECT_SOURCE_DIR}/script/mdc_net_card_1000m_config.sh DESTINATION .)
install(FILES ${PROJECT_SOURCE_DIR}/script/start_hmi.sh DESTINATION .)

file(WRITE ${CMAKE_INSTALL_PREFIX}/os_env "${RUNTIME_OS}")

install(DIRECTORY ${PROJECT_SOURCE_DIR}/configs/ DESTINATION ./conf)

if(UTEST)
    file(COPY ${PROJECT_SOURCE_DIR}/configs/ DESTINATION ${TEST_OUTPUT_PATH}/conf)   
endif(UTEST)


# if(${IS_X86})
#     install(CODE "file(REMOVE \"${CMAKE_INSTALL_PREFIX}/conf/global_conf/icvos_machine.pt\")" BEFORE INSTALL)
#     if(EXISTS ${PROJECT_SOURCE_DIR}/modules/algo/config/x86)
#       install(FILES ${PROJECT_SOURCE_DIR}/modules/algo/config/x86/icvos_machine.pt DESTINATION ./conf/global_conf)
#       install(FILES ${PROJECT_SOURCE_DIR}/modules/algo/config/x86/run.sh DESTINATION .)
#     endif()
#     install(FILES ${PROJECT_SOURCE_DIR}/script/start_hmi.sh DESTINATION .)
# endif()

# if(${ENABLE_MDC_610})
#     if(EXISTS ${PROJECT_SOURCE_DIR}/modules/algo/config/mdc610/mdc_net_card_1000m_config.sh)
#       install(FILES ${PROJECT_SOURCE_DIR}/modules/algo/config/mdc610/mdc_net_card_1000m_config.sh DESTINATION .
#           PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
#                       GROUP_READ GROUP_WRITE GROUP_EXECUTE
#                       WORLD_READ WORLD_WRITE WORLD_EXECUTE
#       )
#     endif()
# endif()

FILE(GLOB_RECURSE PANGU_LIBS
            ${BUILD_OUT_DIR}/lib/*)

install(FILES ${PANGU_LIBS} DESTINATION lib)

if (${RUNTIME_OS} STREQUAL ICVOS)
    foreach(lib_file ${OS_MSG_LIBS})
        get_filename_component(src_f ${lib_file} NAME)
        set(target_f ${CMAKE_INSTALL_PREFIX}/lib/${src_f})
        install(FILES ${lib_file} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
    endforeach()
endif()

if (INSTALL_THIRDPARTY_LIB)
    set(THIRD_LIBS ${THIRD_LIBS} ${ROS_LIBS} ${VSOMEIP_LIBS} ${ABSEIL_LIBS})
    foreach(lib_file ${THIRD_LIBS})
        get_filename_component(src_f ${lib_file} NAME)
        set(target_f ${CMAKE_INSTALL_PREFIX}/lib/${src_f})
        if(NOT EXISTS ${target_f})
            install(FILES ${lib_file} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
        endif()
    endforeach()

    foreach(lib_file ${PROTO_LIBS})
        get_filename_component(src_f ${lib_file} NAME)
        set(target_f ${CMAKE_INSTALL_PREFIX}/lib/${src_f})
        if(NOT EXISTS ${target_f})
            install(FILES ${lib_file} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
        endif()
    endforeach()

    
    foreach(lib_file ${BOOST_LIBS})
        get_filename_component(src_f ${lib_file} NAME)
        set(target_f ${CMAKE_INSTALL_PREFIX}/lib/${src_f})
        if(NOT EXISTS ${target_f})
            install(FILES ${lib_file} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
        endif()
    endforeach()

    foreach(lib_file ${MDC_SYS_LIBS})
        get_filename_component(src_f ${lib_file} NAME)
        set(target_f ${CMAKE_INSTALL_PREFIX}/lib/${src_f})
        if(NOT EXISTS ${target_f})
            install(FILES ${lib_file} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
        endif()
    endforeach()

    foreach(lib_file ${ONNXRUNTIME_LIBS})
        get_filename_component(src_f ${lib_file} NAME)
        set(target_f ${CMAKE_INSTALL_PREFIX}/lib/${src_f})
        if(NOT EXISTS ${target_f})
            install(FILES ${lib_file} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
        endif()
    endforeach()
endif()

if(BUILD_SHADOW)
    install(DIRECTORY configs modules script DESTINATION ${SHADOW_DIR} 
        FILES_MATCHING
            PATTERN "*"
            REGEX "module_lib/.*\\.cc$" EXCLUDE
            REGEX "module_lib/.*\\.c$" EXCLUDE
            REGEX "module_lib/.*\\.cpp$" EXCLUDE
            REGEX "modules/algo/HmiRviz/.*" EXCLUDE)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/common/CMakeLists.txt" COMMON_CMAKE_CONTENT)
    string(FIND "${COMMON_CMAKE_CONTENT}" "install(DIRECTORY ./ DESTINATION \${COMMON_INSTALL_DIR}" INSTALL_FOUND)
    if(INSTALL_FOUND EQUAL -1)
        message(STATUS "未在 dependencies/common/CMakeLists.txt 中发现 install 指令，执行默认安装")
        install(DIRECTORY dependencies/common DESTINATION ${SHADOW_DIR}/dependencies
            FILES_MATCHING
                PATTERN "*"
                PATTERN "os_adapter/icvos/*" EXCLUDE)
    else()
        message(STATUS "已在 dependencies/common/CMakeLists.txt 中定义 install 指令，跳过默认安装")
    endif()
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/user_message")
        install(DIRECTORY dependencies/user_message DESTINATION ${SHADOW_DIR}/dependencies)
    endif()
    install(FILES .gitignore CMakeLists.txt ReadMe.md version.hpp DESTINATION ${SHADOW_DIR})
    # install(DIRECTORY dependencies/module_lib DESTINATION ${SHADOW_DIR}/dependencies)
    install(FILES dependencies/deps.cmake DESTINATION ${SHADOW_DIR}/dependencies)  
endif()
