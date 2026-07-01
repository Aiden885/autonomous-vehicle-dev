
cmake_minimum_required(VERSION 3.16.1)
project(test_ACCModule)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
set(EXECUTABLE_OUTPUT_PATH ${UTEST_BIN_PATH})
set(MY_DEPENDENCY_PATH ${BUILD_OUT_DIRECTORY}/lib)
install(CODE "set(MY_DEPENDENCY_PATH "${MY_DEPENDENCY_PATH}")")
FILE(GLOB_RECURSE TEST_SOURCES *.cpp)
message(STATUS "project_source is ${PROJECT_SOURCES}")
# set_source_files_properties(${PROTO_CPPS} PROPERTIES GENERATED TRUE)
foreach(testfile ${TEST_SOURCES})
    STRING(REGEX REPLACE ".+/(.+)\..*" "\1" test_bin ${testfile})
    add_executable(${test_bin} ${testfile})
    target_link_libraries(${test_bin} ${TARGET_LIBS} gtest ACCModule ACCModule_conf_pb protobuf_tools)
    execute_process(COMMAND mkdir -p ${EXECUTABLE_OUTPUT_PATH}
                    COMMAND echo "${PROJECT_BINARY_DIR}/CMakeFiles/${test_bin}.dir"
                    COMMAND tee ${EXECUTABLE_OUTPUT_PATH}/${test_bin}.covdir)
endforeach()
file(COPY test_data DESTINATION ${EXECUTABLE_OUTPUT_PATH})

if (NOT test_bin STREQUAL "")

    install_depend_libs(${test_bin} ${TEST_LIB_COPY_DIR})
endif()