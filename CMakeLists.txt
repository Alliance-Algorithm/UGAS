# UGAS - Alliance 2022
# Developer(s): 21-THY 19-LYJ
# Latest Update: 2023/04/03

cmake_minimum_required(VERSION 3.0)
project(UGAS VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/release)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

if (CMAKE_BUILD_TYPE STREQUAL Debug)
    ADD_DEFINITIONS(-DDEBUG)
endif ()

include(${PROJECT_SOURCE_DIR}/cmake/utils.cmake)

find_package(OpenCV 4.7 REQUIRED)

option(USE_HIK_CAMERA "turn on this to use hik camera" ON)

if (DEFINED USE_HIK_CAMERA)
    set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
    find_package(HikCameraSDK)
    set(LIB_NAME ${HikCameraSDK_LIB})
else ()
    set(CAMERA_TYPE HT)
    set(LIB_NAME MVSDK)
    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/${CAMERA_TYPE}CameraSDK/${CMAKE_SYSTEM_NAME})
endif ()

iter_directories(${CMAKE_CURRENT_SOURCE_DIR}/src MODULES)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${OpenCV_INCLUDES})

foreach (MODULE ${MODULES})
    message("Add module: " ${MODULE})
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/${MODULE})
endforeach (MODULE)

file(GLOB_RECURSE SRC_LIST src/*.c*)

add_executable(${PROJECT_NAME} main.cpp ${SRC_LIST})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${MODULES} ${HikCameraSDK_LIB})


#################
# Install Part
#################
if (CMAKE_BUILD_TYPE STREQUAL Release)
    include(GNUInstallDirs)

    install(TARGETS ${PROJECT_NAME} ${MODULES}
            EXPORT ${PROJECT_NAME}Targets
            LIBRARY DESTINATION bin
            ARCHIVE DESTINATION lib
            RUNTIME DESTINATION bin
            PUBLIC_HEADER DESTINATION include
            PRIVATE_HEADER DESTINATION include)

    file(GLOB RULES_FILES ${PROJECT_SOURCE_DIR}/rules/*.rules)

    install(FILES ${RULES_FILES} DESTINATION rules)

    install(FILES ${RULES_FILES} DESTINATION /etc/udev/rules.d)

    install(DIRECTORY ${PROJECT_SOURCE_DIR}/docs
            DESTINATION .)

    install(DIRECTORY ${PROJECT_SOURCE_DIR}/models
            DESTINATION .)

    install(FILES ${PROJECT_SOURCE_DIR}/README.md
            DESTINATION .)



    include(CMakePackageConfigHelpers)

    write_basic_package_version_file(
            ${PROJECT_NAME}Config-version.cmake
            VERSION ${PACKAGE_VERSION}
            COMPATIBILITY AnyNewerVersion  # 表示该函数库向下兼容
    )
    install(EXPORT ${PROJECT_NAME}Targets
            FILE ${PROJECT_NAME}Targets.cmake
            DESTINATION cmake
            )

    configure_file(${PROJECT_NAME}Config.cmake.in ${PROJECT_NAME}Config.cmake @ONLY)

    install(FILES
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config-version.cmake
            DESTINATION cmake)
endif ()
