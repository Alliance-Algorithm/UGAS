macro(iter_directories dir subdirectories)
    file(GLOB children ${dir}/*)
    set(subdirectories "")
    foreach(child ${children})
        RELATIVE_PATH(child_name ${dir} ${child})
        if(IS_DIRECTORY ${dir}/${child_name})
            list(APPEND ${subdirectories} ${child_name})
        endif()
    endforeach()

endmacro()
macro(relative_path relative dir absolute)
    file(RELATIVE_PATH ${relative} ${dir} ${absolute})
    endmacro()