cmake_minimum_required(VERSION 3.16.1)
project(main)

# 找到 OpenCV 库
find_package(OpenCV REQUIRED)
# 包含 OpenCV 的头文件目录
include_directories(${OpenCV_INCLUDE_DIRS})

# 设置包含目录
include_directories(./dependence)
include_directories(./src)

# 找到 PCL 库
find_package(PCL 1.10 REQUIRED)
# 添加 PCL 的包含目录
include_directories(${PCL_INCLUDE_DIRS})
# 添加 PCL 的定义和编译选项
add_definitions(${PCL_DEFINITIONS})

# 找到 Protobuf 包
find_package(Protobuf REQUIRED)
# 包含 Protobuf 头文件目录
include_directories(${PROTOBUF_INCLUDE_DIR})

# 定义 Proto 文件
set(PROTO_FILES
    ./proto/perception.proto
    ./proto/cloudData.proto) # 添加此行
# 生成所有 Proto 源文件和头文件
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
# 包含生成的 Proto 文件路径
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# 找到 yaml 库
find_package(yaml-cpp REQUIRED)
# 添加 yaml 的包含目录
include_directories(${YAML_CPP_INCLUDE_DIRS})

# 搜集源文件
file(GLOB_RECURSE SRCS_CPP ./dependence/*.cpp ./src/*.cpp ./test/*.cpp)

# 创建可执行文件
add_executable(${PROJECT_NAME} ${SRCS_CPP} ${PROTO_SRCS} ${PROTO_HDRS})

# 链接 PCL、Protobuf 、OpenCV和yaml 库
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${PROTOBUF_LIBRARY} ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})

