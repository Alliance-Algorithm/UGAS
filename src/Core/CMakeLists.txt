string(REGEX REPLACE ".*/\(.*\)" "\\1" NAME ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB_RECURSE SRC_LIST *.cpp)

add_library(${NAME} SHARED ${SRC_LIST})

target_link_libraries(${NAME} ${OpenCV_LIBS} ThirdParty)