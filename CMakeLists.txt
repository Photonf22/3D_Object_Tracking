cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

add_definitions(-std=c++11)

#set(CXX_FLAGS "-Wall")
set(CXX_FLAGS "-Wall" "-pedantic")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(camera_fusion)

find_package(OpenCV 4.1 REQUIRED)
find_package(Iconv REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED NO_MODULE)
find_package(Qt5Core REQUIRED)
find_package(Qt5Test REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Concurrent)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})
find_package(Ceres REQUIRED)

# Executable for create matrix exercise
add_executable (3D_object_tracking src/camFusion_Student.cpp src/FinalProject_Camera.cpp src/lidarData.cpp src/matching2D_Student.cpp src/objectDetection2D.cpp)
target_link_libraries (3D_object_tracking ${OpenCV_LIBRARIES})
target_link_libraries(3D_object_tracking Qt5::Test)
target_link_libraries(3D_object_tracking Qt5::Concurrent)
target_link_libraries (3D_object_tracking ${OpenCV_LIBS})
