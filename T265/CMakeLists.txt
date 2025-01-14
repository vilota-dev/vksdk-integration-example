cmake_minimum_required(VERSION 3.16)
project(T265_publisher VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the directory to find the necessary libraries.
# Alternatively, you may specify these when you run `cmake` command instead. To do so, run
# `cmake -DCapnProto_DIR="..." -Dvk_sdk_DIR="..." "<source_directory>"` in your build directory.

set(CapnProto_DIR "/opt/vilota/lib/cmake/CapnProto")
set(vk_sdk_DIR "/opt/vilota/lib/cmake/vk_sdk")
# set(librealsense_DIR "/home/vilota/git/librealsense/include/librealsense2")

find_package(vk_sdk CONFIG REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

add_executable(${PROJECT_NAME} odom_publisher.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC CapnProto::capnp PRIVATE vilota::vk_sdk realsense2 Eigen3::Eigen)