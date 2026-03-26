cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(v2xServ)
add_definitions(-std=c++11)
add_definitions(-g)

#set ivics 
file(GLOB LIB_IVICS
    lib/ivics/*.so
     lib/ivics/*.a
)

link_libraries(${LIB_IVICS})

find_package(PkgConfig REQUIRED)
# 定义查找路径
set(Protobuf_PREFIX_PATH "/usr/protobuf")
# 添加到 CMAKE_PREFIX_PATH
list(APPEND CMAKE_PREFIX_PATH "${Protobuf_PREFIX_PATH}")
# 查找 Protobuf
find_package(Protobuf REQUIRED)

#message(WARNING "protobuf info:" "${Protobuf_FOUND} ${Protobuf_VERSION} ${Protobuf_INCLUDE_DIRS} ${Protobuf_LIBRARIES} ${Protobuf_PROTOC_LIBRARIES} ${Protobuf_LITE_LIBRARIES}")


include_directories(
	    ${Protobuf_INCLUDE_DIR}
        /usr/local/include
        ./lib/ivics
         ./include
         ./include/ivics
         ./proto/*.h
)

file(GLOB SRC_FILES
    ./src/*cpp
    ./proto/*.cc
    ./V2xMain.cpp 
)

add_executable(v2xServ ${SRC_FILES})

target_link_libraries(v2xServ ${PROTOBUF_LIBRARIES} ${LIB_IVICS} -lzmq -lpthread)
