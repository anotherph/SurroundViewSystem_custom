cmake_minimum_required(VERSION 3.0)
project(around_view_front)
# project(ex)
include_directories( ${OpenCV_INCLUDE_DIRS} )
# set(CMAKE_REQUIRED_LIBRARIES ${prev_libraries} -lstdc++fs)
find_package( OpenCV REQUIRED )
# set for compiler 
set(CMAKE_CXX_COMPILER g++)

# set CUDA
find_package(CUDA)
include_directories("${CUDA_INCLUDE_DIRS}")
add_definitions(-DUSE_CUDA)

# set for debug
set(CMAKE_BUILD_TYPE Debug)
# set(SOURCES Demo.cpp include/Stitching360.h src/Stitching360.cpp)
set(SOURCES stream_front.cpp include/Stitching360.h src/Stitching360.cpp)
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} "stdc++fs")
