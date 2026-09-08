cmake_minimum_required(VERSION 3.16.1)
project(modules)

FILE(GLOB_RECURSE cmake_files FOLLOW_SYMLINKS RELATIVE ${PROJECT_SOURCE_DIR} "CMakeLists.txt")
SET(module_list "")
FOREACH(f ${cmake_files})
    get_filename_component(dir ${f} DIRECTORY)
    if(NOT("${dir}" MATCHES "(.*)test(.*)" OR "${dir}" STREQUAL ""))
        # MDC610平台下排除SimulationAdapter和HmiRviz
        if(${ENABLE_MDC_610} AND (
            "${dir}" MATCHES "(.*)SimulationAdapter(.*)" OR 
            "${dir}" MATCHES "(.*)HmiRviz(.*)"
            ))
            message(STATUS "skip ${dir} on MDC610 platform.")
            continue()
        endif()

        # J6M平台下排除SimulationAdapter/HmiRviz/HdMapService
        if(${ENABLE_J6} AND (
            "${dir}" MATCHES "(.*)SimulationAdapter(.*)" OR 
            "${dir}" MATCHES "(.*)HmiRviz(.*)" 
            ))
            message(STATUS "skip ${dir} on J6M platform.")
            continue()
        endif()
        # X86平台下排除InsFusion
        if(${IS_X86} AND (
            "${dir}" MATCHES "(.*)InsFusion(.*)" OR
            "${dir}" MATCHES "(.*)SensorModule(.*)"
            ))
            message(STATUS "skip ${dir} on X86 platform.")
            continue()
        endif()
        
        message(STATUS "search ${dir} in modules!")
        LIST(APPEND module_list ${dir}) 
    endif()
ENDFOREACH()

foreach(mod ${module_list})
    if(NOT mod STREQUAL "algo/proto")
        add_module(${mod})
    endif()
endforeach()