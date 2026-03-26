cmake_minimum_required(VERSION 3.16.1)
project(main)

# 设置包含目录
include_directories(./dependence)
include_directories(./src)


# 查找Eigen库
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

# 搜集源文件
file(GLOB_RECURSE SRCS_CPP ./dependence/*.cpp ./src/*.cpp ./test/*.cpp)

# 创建可执行文件
add_executable(${PROJECT_NAME} ${SRCS_CPP})

# 链接库
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
