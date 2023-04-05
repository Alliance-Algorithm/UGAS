# UGAS - Alliance 2022
# Developer(s): 21-THY 19-LYJ
# Latest Update: 2023/04/03

cmake_minimum_required(VERSION 3.0)
project(UGAS VERSION 1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if (CMAKE_BUILD_TYPE STREQUAL Debug)
    ADD_DEFINITIONS(-DDEBUG)
endif ()

include(${PROJECT_SOURCE_DIR}/cmake/utils.cmake)
# OpenCV 库目录
#set(OpenCV_DIR D:/opencv/build)
find_package(OpenCV 4.7 REQUIRED)

ITER_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/src MODULES)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${OpenCV_INCLUDES})
link_directories(/opt/MVS/lib/64)
#foreach (MODULE ${MODULES})
#    message("Add module: " ${MODULE})
#    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/${MODULE})
#endforeach (MODULE)
file(GLOB_RECURSE SRC_LIST src/*.c*)
add_executable(${PROJECT_NAME} main.cpp ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} MvCameraControl)


