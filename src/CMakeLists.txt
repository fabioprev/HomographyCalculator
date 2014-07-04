cmake_minimum_required(VERSION 2.8)
project(HomographyCalculator)

add_definitions(-Wall)
add_definitions(-O3)

find_package(OpenCV REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ../bin)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(HomographyCalculator HomographyCalculator.cpp)
target_link_libraries(HomographyCalculator ${OpenCV_LIBS})
