cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(Planning)
ADD_DEFINITIONS(-std=c++17)
find_package(Eigen3)
#画图
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
#pkg_check_modules(OPENCV REQUIRED opencv)
#end画图

#头文件路径 #画图最后一行库
include_directories (/usr/local/include
		/usr/lib/x86_64-linux-gnu
		./
		./controlMsg
		./imuMsg
		./mapMsg
		./planningMsg
		./predictionMsg
		./uiMsg
		./serverMsg
		./include
		${OpenCV_INCLUDE_DIRS}
)

#源文件集合
file(GLOB_RECURSE
	SRC_FILES
#	./planningFigure.cpp
#	./localPlanning.cpp
	./controlMsg/*.cc
	./imuMsg/*.cc
	./mapMsg/*.cpp
	./planningMsg/*.cc
	./predictionMsg/*.cc
	./uiMsg/*.cc
	./serverMsg/*.cc
	../Thread/ztgeographycoordinatetransform.cpp
	./serverMsg/TrafficLight.cpp
	./perception/*cc
	
	# ./globalPlanning_m.cpp
	# ./globalPlanning.cpp
	# ./interpolate_m.cpp
	# ./localPlanning_m.cpp
	# ./localPlanning.cpp
	# ./planningFigure_m.cpp

	# ./src/*.cpp

	./geometry_m.cpp
	./globalPlanning_m_2.cpp
	./globalPlanning_m.cpp
	./globalPlanning.cpp
	./interpolate_m.cpp
	./localPlanning_m.cpp
	./planningFigure_m.cpp
)

link_directories (/usr/lib/x86_64-linux-gnu)

#可执行文件生成
add_executable(planning
	${SRC_FILES}
)


#外部库获取
find_package(Protobuf REQUIRED)
set( ZMQ_LIBRARIES "-lzmq" )
set( LPTHREAD "-lpthread")


include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

#外部库链接 #画图最后一个库
target_link_libraries(planning ${ZMQ_LIBRARIES} ${LPTHREAD} ${PROTOBUF_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)

set(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g3 -ggdb") #debug模式下开启-g选项
##SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall") #如果set(CMAKE_BUILD_TYPE RELEASE)则使用该配置
