
cmake_minimum_required(VERSION 3.2)

project(geomeasurer)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

find_package(PCL 1.7 REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR True)

include_directories(${PCL_INCLUDE_DIRS})

message("include directory:${CMAKE_SOURCE_DIR}")

message("${CMAKE_INCLUDE_CURRENT_DIR}")

add_executable(testgeomeasurer geomeasurer/extract_geometry_feature/extractgeometryfeature.cpp geomeasurer/sensor/scan_data.cpp geomeasurer/io/scan_reader.cpp main.cpp)

target_link_libraries(testgeomeasurer ${PCL_LIBRARIES})

install(TARGETS testgeomeasurer RUNTIME DESTINATION bin)

add_subdirectory(geomeasurer)

add_subdirectory(geomeasurer_ros)