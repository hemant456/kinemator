cmake_minimum_required(VERSION 2.8.9)
project (kinemator)
include_directories(include ${OpenCV_INCLUDE_DIRS})
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

add_executable(kinemator src/ImagePlanner.cpp src/Kinemator.cpp src/main.cpp src/math.cpp)

target_link_libraries(kinemator opencv_calib3d)