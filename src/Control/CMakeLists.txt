string(REGEX REPLACE ".*/\(.*\)" "\\1" NAME ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB_RECURSE SRC_LIST *.cpp)

add_library(${NAME} ${SRC_LIST})

target_link_libraries(${NAME} ${OpenCV_LIBS} Util Core)
if (CMAKE_BUILD_TYPE STREQUAL Release)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include
        FILES_MATCHING
        PATTERN "*.h*" )
    endif()