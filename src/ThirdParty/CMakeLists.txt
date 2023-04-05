string(REGEX REPLACE ".*/\(.*\)" "\\1" NAME ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB_RECURSE SRC_LIST *.c*)
add_library(${NAME} ${SRC_LIST})
target_link_libraries(${NAME})

install(FILES ${HikCameraSDK_LIB_DIR}/lib${LIB_NAME}.so
        DESTINATION bin)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include
        FILES_MATCHING
        PATTERN "*.h*" )

