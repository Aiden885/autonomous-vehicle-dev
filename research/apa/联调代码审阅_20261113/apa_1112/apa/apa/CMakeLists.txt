cmake_minimum_required(VERSION 3.16.1)

project(apa)
find_package(Threads REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/cmake/ApaVehicleProfile.cmake")
apa_generate_vehicle_profile(APA_GENERATED_PROFILE_DIR)
include_directories(BEFORE "${APA_GENERATED_PROFILE_DIR}")

set(CORE_LIBS zmq)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories(${THIRD_PATH}/include)

set(MOD_NAME ${PROJECT_NAME})
set(APA_OS_PROTO_LIBS ${OS_LIBS})
list(FILTER APA_OS_PROTO_LIBS INCLUDE REGEX "libos_proto_pb\\.so")
message("${PROJECT_NAME} CMakeLists Version is 1.5.2")

if(NOT COMMON_MESSAGE STREQUAL "no")
    set(proto_dir ${PROJECT_SOURCE_DIR}/proto)
    set(PROTO_CPPS "")
    set(PROTO_HEADERS "")
    set(PROTO_INCLUDE_DIR ${COMMON_MSG_PROTO_DIR}/common ${OS_PROTO_DIR})
    generate_proto(${proto_dir} ${PROTOCPP_DIR} PROTO_CPPS PROTO_HEADERS ${PROTO_INCLUDE_DIR})
    add_library(${PROJECT_NAME}_conf_pb SHARED ${PROTO_CPPS})
    target_link_libraries(${PROJECT_NAME}_conf_pb ${CONF_PROTO_LIBS} ${APA_OS_PROTO_LIBS} protobuf)
    install(TARGETS ${PROJECT_NAME}_conf_pb
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
    install_depend_libs(${PROJECT_NAME}_conf_pb ${CMAKE_INSTALL_PREFIX}/lib)
endif()

if(EXISTS ${PROJECT_SOURCE_DIR}/module_lib)
    set(MODULELIB_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/module_lib)
    include_directories(${MODULELIB_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/module_lib/apa_algorithm/apa
        ${PROJECT_SOURCE_DIR}/module_lib/apa_algorithm/XXX/include)
    FILE(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS
        module_lib/*.cc
        module_lib/*.c
        module_lib/*.cpp)
    list(FILTER LIB_SOURCES EXCLUDE REGEX ".*/apa_algorithm/XXX/test/.*")
    if(LIB_SOURCES)
        message(STATUS "detect module_lib of ${PROJECT_NAME}, build ${PROJECT_NAME}_core!")
        add_library(${PROJECT_NAME}_core SHARED ${LIB_SOURCES})
        target_compile_features(${PROJECT_NAME}_core PRIVATE cxx_std_20)
        target_link_libraries(${PROJECT_NAME}_core ${CORE_LIBS} m dl Threads::Threads)
        if(TARGET ${PROJECT_NAME}_conf_pb)
            target_link_libraries(${PROJECT_NAME}_core ${PROJECT_NAME}_conf_pb)
        endif()
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

set(TARGET_LIBS ${CORE_LIBS} ${PROJECT_NAME}_conf_pb ${OS_LIBS})
include_directories(${PROJECT_SOURCE_DIR}/src ${MODULELIB_INCLUDE_DIR})

FILE(GLOB_RECURSE PROJECT_SOURCES
    src/*.c
    src/*.cc
    src/*.cpp)
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*/ApaZmqInputModule\\.(cc|cpp|c)$")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*/ApaZmqOutputModule\\.(cc|cpp|c)$")
add_library(${PROJECT_NAME} SHARED ${PROJECT_SOURCES})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
target_link_libraries(${PROJECT_NAME} ${TARGET_LIBS} user_message_msg_pb Threads::Threads)
add_executable(apa_planner_worker worker/apa_planner_worker.cpp)
target_link_libraries(apa_planner_worker apa_core apa_conf_pb protobuf)
set_target_properties(apa_planner_worker PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
add_dependencies(${PROJECT_NAME} apa_planner_worker)
install(TARGETS apa_planner_worker RUNTIME DESTINATION bin)
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install_depend_libs(${PROJECT_NAME} ${CMAKE_INSTALL_PREFIX}/lib)

FILE(GLOB APAZMQINPUTMODULE_SOURCES
    src/ApaZmqInputModule.cc
    src/ApaZmqInputModule.cpp)
if(APAZMQINPUTMODULE_SOURCES)
    add_library(ApaZmqInputModule SHARED ${APAZMQINPUTMODULE_SOURCES})
    target_link_libraries(ApaZmqInputModule ${TARGET_LIBS} user_message_msg_pb)
    install(TARGETS ApaZmqInputModule
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
    install_depend_libs(ApaZmqInputModule ${CMAKE_INSTALL_PREFIX}/lib)
endif()

FILE(GLOB APAZMQOUTPUTMODULE_SOURCES
    src/ApaZmqOutputModule.cc
    src/ApaZmqOutputModule.cpp)
if(APAZMQOUTPUTMODULE_SOURCES)
    add_library(ApaZmqOutputModule SHARED ${APAZMQOUTPUTMODULE_SOURCES})
    target_link_libraries(ApaZmqOutputModule ${TARGET_LIBS} user_message_msg_pb)
    install(TARGETS ApaZmqOutputModule
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
    install_depend_libs(ApaZmqOutputModule ${CMAKE_INSTALL_PREFIX}/lib)
endif()

if(UTEST)
    set(TARGET_LIBS ${TARGET_LIBS} ${PROJECT_NAME} protobuf_tools)
    set(UTEST_BIN_PATH ${TEST_OUTPUT_PATH}/bin/${MOD_NAME}/)
endif(UTEST)

install(DIRECTORY conf/ DESTINATION conf/node_module/${PROJECT_NAME})
install(FILES README.md DESTINATION share/apa)
install(DIRECTORY docs/ DESTINATION share/apa/docs FILES_MATCHING PATTERN "*.md")
