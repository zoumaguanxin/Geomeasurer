
cmake_minimum_required(VERSION 3.2)

project(geomeasurer)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

find_package(falkolib REQUIRED)

find_package(FLIRT REQUIRED)

find_package(PCL 1.7 REQUIRED)

find_package(OpenCV REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR True)

include_directories(${PCL_INCLUDE_DIRS} ${falkolib_INCLUDE_DIRS} ${OpenCV2_INCLUDE_DIRS} ${FLIRT_INCLUDE_DIRS})

message("include directory:${CMAKE_SOURCE_DIR}")

message("${CMAKE_INCLUDE_CURRENT_DIR}") 

set(LIB_SRC geomeasurer/transform/pose2d.cpp geomeasurer/common/math_supplement.cpp geomeasurer/extract_geometry_feature/extractgeometryfeature.cpp geomeasurer/sensor/scan_data.cpp geomeasurer/io/scan_reader.cpp)

#add_library(gfs ${LIB_SRC})

#target_link_libraries(gfs ${PCL_LIBRARIES})

#add_executable(testgeomeasurer geomeasurer/extract_geometry_feature/extractgeometryfeature.cpp geomeasurer/sensor/scan_data.cpp geomeasurer/io/scan_reader.cpp main.cpp)

add_executable(testgfs geomeasurer/extract_geometry_feature/extract_geometry_feature_main.cpp ${LIB_SRC})

add_executable(test_falko geomeasurer/extract_geometry_feature/FAKLO_test.cpp  ${LIB_SRC})

add_executable(dalko_testlog geomeasurer/extract_geometry_feature/dalko_testlog.cpp  ${LIB_SRC})

#target_link_libraries(testgeomeasurer ${PCL_LIBRARIES})

#target_link_libraries(testgfs gfs)

target_link_libraries(testgfs ${PCL_LIBRARIES} ${falkolib_LIBRARIES} ${OpenCV_LIBS} ${FLIRT_LIBRARIES})
target_link_libraries(test_falko ${PCL_LIBRARIES} ${falkolib_LIBRARIES} ${OpenCV_LIBS} ${FLIRT_LIBRARIES})
target_link_libraries(dalko_testlog ${PCL_LIBRARIES} ${falkolib_LIBRARIES} ${OpenCV_LIBS} ${FLIRT_LIBRARIES})

#install(TARGETS testgeomeasurer RUNTIME DESTINATION bin)

add_subdirectory(geomeasurer)

add_subdirectory(geomeasurer_ros)