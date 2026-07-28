cmake_minimum_required(VERSION 3.16.1)

project(ACCModule)

set(CORE_LIBS zmq)
include_directories(${THIRD_PATH}/include)

set(MOD_NAME ${PROJECT_NAME})
message("${PROJECT_NAME} CMakeLists Version is 1.5.2")

if(NOT COMMON_MESSAGE STREQUAL "no")
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
        target_compile_options(${PROJECT_NAME}_core PRIVATE -std=c++2a -fconcepts)
        target_link_libraries(${PROJECT_NAME}_core ${CORE_LIBS})
        set(CORE_LIBS ${CORE_LIBS} ${PROJECT_NAME}_core)
        install(TARGETS ${PROJECT_NAME}_core
                LIBRARY DESTINATION lib
                ARCHIVE DESTINATION lib)
        install_depend_libs(${PROJECT_NAME}_core ${CMAKE_INSTALL_PREFIX}/lib)
        if(${BUILD_SHADOW})
            STRING(REGEX REPLACE ".*(modules/.+/${PROJECT_NAME}).*" "\\1" module_path ${PROJECT_SOURCE_DIR})
            install(TARGETS ${PROJECT_NAME}_core
                    LIBRARY DESTINATION ${SHADOW_DIR}/${module_path}/module_lib/lib/${TARGET_PLATFORM}
                    ARCHIVE DESTINATION ${SHADOW_DIR}/${module_path}/module_lib/lib/${TARGET_PLATFORM})
        endif()
    elseif(EXISTS ${PROJECT_SOURCE_DIR}/module_lib/lib/${TARGET_PLATFORM}/lib${PROJECT_NAME}_core.so)
      set(CORE_LIBS ${CORE_LIBS} ${PROJECT_SOURCE_DIR}/module_lib/lib/${TARGET_PLATFORM}/lib${PROJECT_NAME}_core.so)
      install(FILES ${PROJECT_SOURCE_DIR}/module_lib/lib/${TARGET_PLATFORM}/lib${PROJECT_NAME}_core.so DESTINATION lib)
    endif()
endif()

set(TARGET_LIBS  ${CORE_LIBS} ${PROJECT_NAME}_conf_pb ${OS_LIBS})
include_directories(${PROJECT_SOURCE_DIR}/src ${MODULELIB_INCLUDE_DIR})
add_library(${PROJECT_NAME} SHARED src/ACCModule.cc)
target_compile_options(${PROJECT_NAME} PRIVATE -std=c++2a -fconcepts)
target_link_libraries(${PROJECT_NAME} ${TARGET_LIBS} user_message_msg_pb)
install(TARGETS ${PROJECT_NAME}
                LIBRARY DESTINATION lib
                ARCHIVE DESTINATION lib)
install_depend_libs(${PROJECT_NAME} ${CMAKE_INSTALL_PREFIX}/lib)

add_library(ZmqBridgeModule SHARED src/ZmqBridgeModule.cc)
target_compile_options(ZmqBridgeModule PRIVATE -std=c++2a -fconcepts)
target_link_libraries(ZmqBridgeModule ${TARGET_LIBS} user_message_msg_pb)
install(TARGETS ZmqBridgeModule
                LIBRARY DESTINATION lib
                ARCHIVE DESTINATION lib)
install_depend_libs(ZmqBridgeModule ${CMAKE_INSTALL_PREFIX}/lib)

if(UTEST)
    set(TARGET_LIBS ${TARGET_LIBS} ${PROJECT_NAME} protobuf_tools)
    set(UTEST_BIN_PATH ${TEST_OUTPUT_PATH}/bin/${MOD_NAME}/)
    add_subdirectory(test)
    file(COPY conf DESTINATION ${UTEST_BIN_PATH})
endif(UTEST)

install(DIRECTORY conf/ DESTINATION conf/node_module/${PROJECT_NAME})
