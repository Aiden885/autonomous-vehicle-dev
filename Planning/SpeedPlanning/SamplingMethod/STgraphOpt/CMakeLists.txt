cmake_minimum_required(VERSION 3.16.1)
project(STgraphOpt)

include_directories(./src)

add_library(libSTgraphOpt SHARED ./src/STgraphOpt.cpp)
install(TARGETS libSTgraphOpt DESTINATION ./lib)

add_executable(${PROJECT_NAME} ./test/main.cpp)
target_link_libraries (${PROJECT_NAME} ${OpenCV_LIBS} libSTgraphOpt)
