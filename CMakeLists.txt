# UGAS - Alliance 2022
# Developer(s): 21-THY
# Latest Update: 2022/10/20

cmake_minimum_required(VERSION 3.0)
project(UGAS VERSION 1.0 LANGUAGES C CXX)
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# OpenCV 库目录
#set(OpenCV_DIR D:/opencv/build)
find_package(OpenCV 4 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/Parameters)
include_directories(${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/lib/GimbalSerial)
include_directories(${PROJECT_SOURCE_DIR}/lib/ArmorFinder)
include_directories(${PROJECT_SOURCE_DIR}/lib/Common)
include_directories(${PROJECT_SOURCE_DIR}/lib/ImgCapture)
include_directories(${PROJECT_SOURCE_DIR}/lib/ImgPretreat)
include_directories(${PROJECT_SOURCE_DIR}/lib/TargetSolution)
include_directories(${PROJECT_SOURCE_DIR}/lib/TrackingStrategy)
include_directories(${PROJECT_SOURCE_DIR}/lib/Trajectory)
include_directories(${PROJECT_SOURCE_DIR}/third-party)
include_directories(${Opencv_INCLUDE_DIRS})

link_directories(${PROJECT_SOURCE_DIR}/third-party/HTCameraSDK/Linux)
# main sources
aux_source_directory(${PROJECT_SOURCE_DIR} MAIN_SOURCES)

# lib sources
file(GLOB_RECURSE LIB_SOURCES ${PROJECT_SOURCE_DIR}/lib/*/*.c*)

# parameter sources
file(GLOB_RECURSE PARA_SOURCES ${PROJECT_SOURCE_DIR}/Parameters/*.c*)

# generate exec
add_executable(ugas ${MAIN_SOURCES} ${PARA_SOURCES} ${LIB_SOURCES})

# link to OpenCV lib
target_link_libraries(ugas ${OpenCV_LIBS} MVSDK)

