# 1. 最低CMake版本要求
cmake_minimum_required(VERSION 3.16)

# 2. 项目名称 + 同时启用 C 和 C++
project(MainInclude LANGUAGES C CXX)

# 3. 核心配置：输出目录（使用 CMAKE_BINARY_DIR，支持 out-of-source 构建）
set(OUTPUT_DIR ${CMAKE_BINARY_DIR}/lib)
set(BIN_OUTPUT_DIR ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_OUTPUT_DIR})

# 4. C & C++ 标准
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 5. 编译选项（C 和 C++ 分开）
if(MSVC)
    set(C_FLAGS_LIST /W4 /O2 /Zi /DLIB_EXPORT)
    set(CXX_FLAGS_LIST /W4 /O2 /Zi /DLIB_EXPORT /std:c++2a /permissive-)
else()
    set(C_FLAGS_LIST -std=c99 -Wall -Wextra -fPIC)
    set(CXX_FLAGS_LIST -std=c++2a -fconcepts -Wall -Wextra -fPIC)
endif()

if(MODULE_SIM_SCOPE_INC)
    separate_arguments(MODULE_SIM_SCOPE_INC_LIST NATIVE_COMMAND "${MODULE_SIM_SCOPE_INC}")
else()
    set(MODULE_SIM_SCOPE_INC_LIST "")
endif()

# 6. 数学库
if(NOT MSVC)
    find_library(MATH_LIB m)
    set(MATH_LIBRARIES ${MATH_LIB})
else()
    set(MATH_LIBRARIES "")
endif()

# 7. 收集所有目标文件
set(ALL_OBJECT_TARGETS "")
set(ALL_OBJECT_FILES "")

# 8. 拷贝 .o 文件
file(GLOB ATOMIC_OBJS ${CMAKE_CURRENT_SOURCE_DIR}/atomic_lib/*.o)
if(ATOMIC_OBJS)
    file(MAKE_DIRECTORY ${OUTPUT_DIR})
    foreach(OBJ_FILE ${ATOMIC_OBJS})
        get_filename_component(OBJ_NAME ${OBJ_FILE} NAME)
        add_custom_command(
            OUTPUT ${OUTPUT_DIR}/${OBJ_NAME}
            COMMAND ${CMAKE_COMMAND} -E copy ${OBJ_FILE} ${OUTPUT_DIR}/
            COMMENT "Copying ${OBJ_NAME}"
        )
        list(APPEND ALL_OBJECT_FILES ${OUTPUT_DIR}/${OBJ_NAME})
    endforeach()
endif()

# 9. 遍历所有子目录，自动编译 .c 和 .cpp
file(GLOB SUBDIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *)
foreach(SUBDIR ${SUBDIRS})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR})

        # 编译 C++ 文件
        file(GLOB CPP_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}/*.cpp)
        foreach(CPP_FILE ${CPP_FILES})
            get_filename_component(TARGET_NAME ${CPP_FILE} NAME_WE)
            set(OBJ_FILE ${OUTPUT_DIR}/${TARGET_NAME}.o)

            add_custom_command(
                OUTPUT ${OBJ_FILE}
                COMMAND ${CMAKE_CXX_COMPILER}
                    -c ${CPP_FILE}
                    -o ${OBJ_FILE}
                    -I${CMAKE_CURRENT_SOURCE_DIR}
                    -I${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}
                    ${MODULE_SIM_SCOPE_INC_LIST}
                    ${CXX_FLAGS_LIST}
                DEPENDS ${CPP_FILE}
                COMMENT "Compiling ${TARGET_NAME}.cpp (C++)"
            )
            add_custom_target(${TARGET_NAME}_obj ALL DEPENDS ${OBJ_FILE})
            list(APPEND ALL_OBJECT_TARGETS ${TARGET_NAME}_obj)
            list(APPEND ALL_OBJECT_FILES ${OBJ_FILE})
        endforeach()

        # 编译 C 文件
        file(GLOB C_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}/*.c)
        foreach(C_FILE ${C_FILES})
            get_filename_component(TARGET_NAME ${C_FILE} NAME_WE)
            set(OBJ_FILE ${OUTPUT_DIR}/${TARGET_NAME}.o)

            add_custom_command(
                OUTPUT ${OBJ_FILE}
                COMMAND ${CMAKE_C_COMPILER}
                    -c ${C_FILE}
                    -o ${OBJ_FILE}
                    -I${CMAKE_CURRENT_SOURCE_DIR}
                    -I${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}
                    ${MODULE_SIM_SCOPE_INC_LIST}
                    ${C_FLAGS_LIST}
                DEPENDS ${C_FILE}
                COMMENT "Compiling ${TARGET_NAME}.c (C)"
            )
            add_custom_target(${TARGET_NAME}_c_obj ALL DEPENDS ${OBJ_FILE})
            list(APPEND ALL_OBJECT_TARGETS ${TARGET_NAME}_c_obj)
            list(APPEND ALL_OBJECT_FILES ${OBJ_FILE})
        endforeach()
    endif()
endforeach()

# 10. 生成静态库
if(ALL_OBJECT_FILES)
    add_library(MainInclude_static STATIC ${ALL_OBJECT_FILES})
    target_include_directories(MainInclude_static PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    set_target_properties(MainInclude_static PROPERTIES
        LINKER_LANGUAGE CXX
        CXX_STANDARD 17
        C_STANDARD 99
    )
    if(ALL_OBJECT_TARGETS)
        add_dependencies(MainInclude_static ${ALL_OBJECT_TARGETS})
    endif()
endif()

# 11. 动态库
if(ALL_OBJECT_FILES)
    add_library(MainInclude_shared SHARED ${ALL_OBJECT_FILES})
    target_include_directories(MainInclude_shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    set_target_properties(MainInclude_shared PROPERTIES
        LINKER_LANGUAGE CXX
        CXX_STANDARD 17
        C_STANDARD 99
    )
    if(ALL_OBJECT_TARGETS)
        add_dependencies(MainInclude_shared ${ALL_OBJECT_TARGETS})
    endif()
endif()

# 自动检测并编译可执行文件
file(GLOB SUBDIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} */)
foreach(SUBDIR ${SUBDIRS})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR} AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}/main.cpp)
        # 生成可执行文件，以目录名命名
        add_executable(${SUBDIR} ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}/main.cpp)
        # 添加父目录作为头文件搜索路径
        target_include_directories(${SUBDIR} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        # 编译顺序：先编译库，再链接可执行文件
        add_dependencies(${SUBDIR} MainInclude_static MainInclude_shared)
        # 链接静态库或动态库（二选一，推荐静态库避免动态库依赖问题）
        target_link_libraries(${SUBDIR} PRIVATE MainInclude_static ${MATH_LIBRARIES})
        # 设置输出到bin目录
        set_target_properties(${SUBDIR} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${BIN_OUTPUT_DIR})
    endif()
endforeach()

# 自动检测并编译可执行文件
file(GLOB SUBDIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} */)
foreach(SUBDIR ${SUBDIRS})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR} AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}/main.c)
        # 生成可执行文件，以目录名命名
        add_executable(${SUBDIR} ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}/main.c)
        # 添加父目录作为头文件搜索路径
        target_include_directories(${SUBDIR} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        # 编译顺序：先编译库，再链接可执行文件
        add_dependencies(${SUBDIR} MainInclude_static MainInclude_shared)
        # 链接静态库或动态库（二选一，推荐静态库避免动态库依赖问题）
        target_link_libraries(${SUBDIR} PRIVATE MainInclude_static ${MATH_LIBRARIES})
        # 设置输出到bin目录
        set_target_properties(${SUBDIR} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${BIN_OUTPUT_DIR})
    endif()
endforeach()