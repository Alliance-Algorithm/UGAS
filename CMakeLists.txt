# UGAS - Alliance 2022
# Developer(s): 21-THY
# Latest Update: 2022/10/20

cmake_minimum_required(VERSION 3.0)
project(UGAS VERSION 1.0 LANGUAGES C CXX)
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# OpenCV 库目录
set(OpenCV_DIR D:/opencv/build)
find_package(OpenCV 4 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/UGAS)
include_directories(${Opencv_INCLUDE_DIRS})

# main sources
aux_source_directory(UGAS MAIN_SOURCES)

# lib sources
file(GLOB_RECURSE LIB_SOURCES  UGAS/lib/*.c*)

# parameter sources
file(GLOB_RECURSE PARA_SOURCES UGAS/Parameters/*.c*)

# generate exec
add_executable(ugas ${MAIN_SOURCES} ${PARA_SOURCES} ${LIB_SOURCES})

# link to OpenCV lib
target_link_libraries(ugas ${OpenCV_LIBS})
