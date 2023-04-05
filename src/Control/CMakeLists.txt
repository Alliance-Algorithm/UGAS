string(REGEX REPLACE ".*/\(.*\)" "\\1" NAME ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB_RECURSE SRC_LIST *.cpp)

add_library(${NAME} SHARED ${SRC_LIST})
if (CMAKE_SYSTEM_NAME MATCHES "Linux")
    link_directories(${PROJECT_SOURCE_DIR}/src/ThirdParty/HikCameraSDK/Linux)
elseif (CMAKE_SYSTEM_NAME MATCHES "Windows")
    link_directories(${PROJECT_SOURCE_DIR}/src/ThirdParty/HikCameraSDK/Windows)
endif ()
link_libraries(/home/soulde/UGAS/src/ThirdParty/HikCameraSDK/Linux/libMvCameraControl.so)
target_link_libraries(${NAME} ${OpenCV_LIBS} Util)