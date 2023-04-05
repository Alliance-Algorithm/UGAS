if (DEFINED ENV{MVCAM_SDK_PATH})
    set(SDK_ROOT $ENV{MVCAM_SDK_PATH})
    set(HikCameraSDK_Found True)
else ()
    message(FATAL_ERROR "Do Not Found SDK. Please Install SDK And Run 'set_env_path.sh' To Set Environment Variable.")
endif ()
file(GLOB HiKCameraSDK_INCLUDES ${SDK_ROOT}/include/*.h)

if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(HikCameraSDK_LIB_DIR ${SDK_ROOT}/lib/64)

else ()
    set(HikCameraSDK_LIB_DIR ${SDK_ROOT}/lib/32)
endif ()
link_directories(${HikCameraSDK_LIB_DIR})
set(HikCameraSDK_LIB MvCameraControl)


