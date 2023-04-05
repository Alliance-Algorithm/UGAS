string(REGEX REPLACE ".*/\(.*\)" "\\1" NAME ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB_RECURSE SRC_LIST *.c*)
add_library(${NAME} ${SRC_LIST})