
cmake_minimum_required(VERSION 3.16.1)
project(main)   #project为模块名称，与模块目录名一致，在自动生成工具不具备的情况下，工程名要修改
#以下内容中为加可修改注释的都是模板，不要修改或增加新的内容
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")
include_directories(./src)
file(GLOB_RECURSE SRCS_CPP  ./src/*.cpp ./test/*.cpp)


# 找到 Protobuf 包
find_package(Protobuf REQUIRED)
find_package(Threads)
# 包含 Protobuf 头文件目录
include_directories(${PROTOBUF_INCLUDE_DIRS})

# 定义 Proto 文件
set(PROTO_FILES
    ./proto/prediction.proto) # 添加此行
# 生成所有 Proto 源文件和头文件
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
# 包含生成的 Proto 文件路径
include_directories(${CMAKE_CURRENT_BINARY_DIR} )

# 创建可执行文件
add_executable(${PROJECT_NAME} ${SRCS_CPP} ${PROTO_SRCS} ${PROTO_HDRS})

# 链接 Protobuf 库
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})