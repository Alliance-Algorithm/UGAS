cmake_minimum_required(VERSION 3.12)
project(UGAS VERSION 1.0 LANGUAGES C CXX)


configure_file (
    "${PROJECT_SOURCE_DIR}/src/config.h.in"
    "${PROJECT_BINARY_DIR}/config/config.h"
)


set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)


option (ENABLE_DEBUG_CANVAS "Enable debug canvas to draw debug image")
option (ENABLE_RECORDING "Enable recording of raw camera image")
option (ENABLE_OPENVINO "Enable openvino to identify buff" OFF)
option (ENABLE_ROS "Enable ROS to visualize positions" OFF)


find_package(OpenCV 4.7 REQUIRED)
find_package(HikCameraSDK)

if (ENABLE_OPENVINO)
    find_package(OpenVINO REQUIRED)
    set(OpenVINO_LIB openvino::runtime)
endif (ENABLE_OPENVINO)


include_directories(${PROJECT_SOURCE_DIR}/src ${PROJECT_BINARY_DIR}/config)

file(GLOB_RECURSE UGAS_SOURCE CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/*.cc
)


add_executable(${PROJECT_NAME} ${UGAS_SOURCE})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${HikCameraSDK_LIB} ${OpenVINO_LIB} -lpthread)

