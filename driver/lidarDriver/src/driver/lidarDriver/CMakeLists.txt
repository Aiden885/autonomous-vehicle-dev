cmake_minimum_required(VERSION 3.0)
project(livox)

find_package(PCL 1.8 REQUIRED)

add_definitions(${PCL_DEFINITIONS})

find_package(PkgConfig)
pkg_check_modules(APR apr-1)

add_subdirectory(sdk_core)

get_property(communicator_LIB_INCLUDE_DIR GLOBAL PROPERTY "communicator_INCLUDE_DIR")

add_library(livox_shared SHARED livox.cpp lvx_file.cpp lvx_file.h livox.h)
target_link_libraries(livox_shared
        PRIVATE
        ${PROJECT_NAME}_static
        ${PCL_LIBRARIES}
        ${APR_LIBRARIES}
        communicator
        )
target_include_directories(livox_shared PRIVATE communicator_LIB_INCLUDE_DIR ${PCL_INCLUDE_DIRS} ${APR_INCLUDE_DIRS})
       
add_executable(livox main.cpp)
target_link_libraries(livox livox_shared)

# target_link_libraries(livox_shared
# PRIVATE
# ${PROJECT_NAME}_static
# ${PCL_COMMON_LIBRARIES}
# ${PCL_IO_LIBRARIES}
# ${PCL_SEGMENTATION_LIBRARIES}
# ${PCL_VISUALIZATION_LIBRARIES}
# ${PCL_RESISTRATION_LIBRARIES}
# ${PCL_FILTERS_LIBRARIES}
# ${PCL_FEATURES_LIBRARIES}
# ${Boost_LIBRARIES}
# ${Boost_FILESYSTEM_LIBRARY}
# ${Boost_SYSTEM_LIBRARY}
# communicator
# )
