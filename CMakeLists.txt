cmake_minimum_required(VERSION 3.12)
project(ugas VERSION 1.0 LANGUAGES C CXX)

# Set compilation flags
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Initialize custom options
option (ENABLE_DEBUG_CANVAS "Enable debug canvas to draw debug image" OFF)
option (ENABLE_RECORDING "Enable recording of raw camera image" OFF)
option (ENABLE_OPENVINO "Enable openvino to identify buff" OFF)
option (ENABLE_ROS "Enable ROS to visualize positions" OFF)

# Set configure_file
set (CONFIGURE_DIR_PATH ${PROJECT_SOURCE_DIR}/build/config)
configure_file (
        "${PROJECT_SOURCE_DIR}/src/config.h.in"
        "${CONFIGURE_DIR_PATH}/config.h")

# Set the output executable file name:
# When compiling in the ROS environment, the output executable file name will be the node name.
# Otherwise, the output file name will be the project name.
if (ENABLE_ROS)
    set(EXECUTABLE_NAME main)
else ()
    set(EXECUTABLE_NAME ${PROJECT_NAME})
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
endif()

# Find non-ros packages
find_package(OpenCV 4.7 REQUIRED)
find_package(HikCameraSDK REQUIRED)
find_package(Ceres REQUIRED)

if (ENABLE_OPENVINO)
    find_package(OpenVINO REQUIRED)
    set(OpenVINO_LIB openvino::runtime)
endif (ENABLE_OPENVINO)

# Include project source directory
include_directories(${PROJECT_SOURCE_DIR}/src ${CONFIGURE_DIR_PATH})

# Recursively search for all source files under the 'src' folder and store them into UGAS_SOURCE variable
# Flag 'CONFIGURE_DEPENDS' asks cmake to detect GLOB result changes so no need to rerun cmake when adding a new source file.
file(GLOB_RECURSE UGAS_SOURCE CONFIGURE_DEPENDS
        ${PROJECT_SOURCE_DIR}/src/*.cpp
        ${PROJECT_SOURCE_DIR}/src/*.cc)

# Find ros packages & add source files to compilation
if (ENABLE_ROS)
    find_package (ament_cmake_auto REQUIRED)
    ament_auto_find_build_dependencies ()
    ament_auto_add_executable(${EXECUTABLE_NAME} ${UGAS_SOURCE})
else()
    add_executable(${EXECUTABLE_NAME} ${UGAS_SOURCE})
endif()

# Link libraries
target_link_libraries(${EXECUTABLE_NAME} ${OpenCV_LIBS} ${HikCameraSDK_LIB} ${CERES_LIBRARIES} ${OpenVINO_LIB} -lpthread)

# Install package
if (ENABLE_ROS)
    ament_auto_package()
endif()

# Hide warning(s)
add_definitions(-w)
