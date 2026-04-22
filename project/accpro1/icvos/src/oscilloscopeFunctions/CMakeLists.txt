# 1. 最低CMake版本要求（3.10+，兼容大部分环境）
cmake_minimum_required(VERSION 3.10)

# 2. 项目名称
project(accpro1)

# 3. 核心配置：输出目录（.o + 静态库 + 动态库 均在 build/lib/）
set(OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/lib)
set(BIN_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)  # 可执行文件目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_DIR})  # 静态库（.a/.lib）
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR})  # 动态库（.so/.dll）
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_OUTPUT_DIR})  # 可执行文件

# 4. 编译选项（列表形式，兼容GCC/Clang/MSVC）
set(CMAKE_C_STANDARD 99)
if(MSVC)
    # Windows MSVC 编译器选项
    set(CMAKE_C_FLAGS_LIST
        /W4        # 开启高等级警告（对应GCC的-Wall -Wextra）
        /O2        # 优化（按需启用）
        /Zi        # 生成调试信息（按需启用）
        /DLIB_EXPORT # 动态库导出宏（Windows必需，后续说明）
    )
else()
    # Linux/macOS GCC/Clang 编译器选项
    set(CMAKE_C_FLAGS_LIST
        -Wall        # 开启所有警告
        -Wextra      # 额外警告
        -fPIC        # 生成位置无关代码（动态库必需）
        # -O2         # 优化（按需启用）
        # -g          # 调试信息（按需启用）
    )
endif()

# 5. 可选功能：是否生成可执行文件（默认关闭，需手动开启）
#option(BUILD_EXECUTABLE "是否生成可执行文件（需指定主函数文件）" OFF)
#set(MAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/main.c CACHE PATH "可执行文件的主函数文件路径")
# 5. 数学库配置：检测并链接数学库
if(NOT MSVC)
    # 在非Windows系统上，查找并链接数学库
    find_library(MATH_LIB m)
    if(MATH_LIB)
        set(MATH_LIBRARIES ${MATH_LIB})
    else()
        set(MATH_LIBRARIES "")
    endif()
else()
    # Windows上不需要单独链接数学库
    set(MATH_LIBRARIES "")
endif()

# 6. 存储所有自定义目标和文件路径的变量
set(ALL_OBJECT_TARGETS "")
set(ALL_OBJECT_FILES "")

# 6.5 拷贝atomic_lib中的.o文件到build/lib/
file(GLOB ATOMIC_OBJS ${CMAKE_CURRENT_SOURCE_DIR}/atomic_lib/*.o)
if(ATOMIC_OBJS)
    # 创建目标目录
    file(MAKE_DIRECTORY ${OUTPUT_DIR})
    
    # 拷贝所有.o文件
    foreach(OBJ_FILE ${ATOMIC_OBJS})
        get_filename_component(OBJ_NAME ${OBJ_FILE} NAME)
        add_custom_command(
            OUTPUT ${OUTPUT_DIR}/${OBJ_NAME}
            COMMAND ${CMAKE_COMMAND} -E copy ${OBJ_FILE} ${OUTPUT_DIR}/
            COMMENT "Copying ${OBJ_NAME} to ${OUTPUT_DIR}"
        )
        
        # 添加到编译目标
        list(APPEND ALL_OBJECT_FILES ${OUTPUT_DIR}/${OBJ_NAME})
    endforeach()
endif()

# 7. 自动遍历所有子目录（GetRealAsg、TmpTgtCalc、TmpTgtCondCalc）
file(GLOB SUBDIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *)
foreach(SUBDIR ${SUBDIRS})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR})
        # 查找当前子目录下的所有C文件
        file(GLOB C_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}/*.c)
        #list(REMOVE_ITEM C_FILES "${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}/main.c")
        
        foreach(C_FILE ${C_FILES})
            # 提取目标名（如GetRealAsg.c→GetRealAsg）
            get_filename_component(TARGET_NAME ${C_FILE} NAME_WE)
            # .o文件路径：build/lib/目标名.o（所有.o集中存放）
            set(OBJ_FILE ${OUTPUT_DIR}/${TARGET_NAME}.o)
            
            # 8. 手动编译C文件生成.o（输出到lib目录，兼容静态库/动态库）
            add_custom_command(
                OUTPUT ${OBJ_FILE}
                COMMAND ${CMAKE_C_COMPILER}
                -c ${C_FILE}
                -o ${OBJ_FILE}
                -I${CMAKE_CURRENT_SOURCE_DIR}          # 根目录头文件（TmpTgtCalc.h）
                -I${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR} # 子目录私有头文件
                ${CMAKE_C_FLAGS_LIST}
                DEPENDS ${C_FILE}
                COMMENT "Compiling ${TARGET_NAME}.c → ${OBJ_FILE}"
            )
            
            # 9. 创建自定义目标（让CMake管理编译顺序）
            add_custom_target(
                ${TARGET_NAME}_obj
                ALL
                DEPENDS ${OBJ_FILE}
            )
            
            # 10. 记录目标和.o文件路径（用于后续库编译）
            list(APPEND ALL_OBJECT_TARGETS ${TARGET_NAME}_obj)
            list(APPEND ALL_OBJECT_FILES ${OBJ_FILE})
        endforeach()
    endif()
endforeach()

# 11. 生成静态库（.a/.lib）
if(ALL_OBJECT_FILES)
    add_library(
        accpro1_static  # 静态库名（后缀_static区分，可自定义）
        STATIC         # 类型：静态库
        ${ALL_OBJECT_FILES}
    )
    # 显式指定链接语言为C
    set_target_properties(accpro1_static PROPERTIES LINKER_LANGUAGE C)
    # 编译顺序：先生成所有.o，再编译静态库
    if (ALL_OBJECT_TARGETS)
        add_dependencies(accpro1_static ${ALL_OBJECT_TARGETS})
    endif()
    message(STATUS "将生成静态库：${OUTPUT_DIR}/libaccpro1_static.a")  # Linux/macOS示例
else()
    message(WARNING "No C files found in subdirectories, skip building libraries.")
endif()

# 12. 生成动态库（.so/.dll）
if(ALL_OBJECT_FILES)
    add_library(
        accpro1_shared  # 动态库名（后缀_shared区分，可自定义）
        SHARED         # 类型：动态库
        ${ALL_OBJECT_FILES}
    )
    # 显式指定链接语言为C
    set_target_properties(accpro1_shared PROPERTIES LINKER_LANGUAGE C)
    # 编译顺序：先生成所有.o，再编译动态库
    if (ALL_OBJECT_TARGETS)
        add_dependencies(accpro1_shared ${ALL_OBJECT_TARGETS})
    endif()

    # 动态库额外配置（跨平台兼容）
    if(MSVC)
        # Windows：动态库导出宏（需在头文件中配合使用，否则无法外部调用）
        target_compile_definitions(accpro1_shared PRIVATE LIB_EXPORT)
        message(STATUS "将生成动态库：${OUTPUT_DIR}/accpro1_shared.dll")
    else()
        # Linux/macOS：设置动态库版本（可选）
        set_target_properties(accpro1_shared PROPERTIES
            VERSION 1.0.0  # 版本号
            SOVERSION 1    # 兼容版本号
        )
        message(STATUS "将生成动态库：${OUTPUT_DIR}/libaccpro1_shared.so.1.0.0")
    endif()
endif()

# 自动检测并编译可执行文件
file(GLOB SUBDIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} */)
foreach(SUBDIR ${SUBDIRS})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR} AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}/main.c)
        # 生成可执行文件，以目录名命名
        add_executable(${SUBDIR} ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}/main.c)
        # 添加父目录作为头文件搜索路径
        target_include_directories(${SUBDIR} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        # 编译顺序：先编译库，再链接可执行文件
        add_dependencies(${SUBDIR} accpro1_static accpro1_shared)
        # 链接静态库或动态库（二选一，推荐静态库避免动态库依赖问题）
        target_link_libraries(${SUBDIR} PRIVATE accpro1_static ${MATH_LIBRARIES})
        # 设置输出到bin目录
        set_target_properties(${SUBDIR} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${BIN_OUTPUT_DIR})
    endif()
endforeach()

# 13. 可选功能：生成可执行文件（需手动开启BUILD_EXECUTABLE=ON）
#if(BUILD_EXECUTABLE)
#    if(EXISTS ${MAIN_FILE})
#        add_executable(
#            my_executable  # 可执行文件名（可自定义）
#            ${MAIN_FILE}   # 主函数文件（main.c）
#        )
#        # 编译顺序：先编译库，再链接可执行文件
#        add_dependencies(my_executable accpro1_static accpro1_shared)
#        # 链接静态库或动态库（二选一，推荐静态库避免动态库依赖问题）
#        target_link_libraries(my_executable PRIVATE accpro1_static)
#        # 显式指定链接语言为C
#        set_target_properties(my_executable PROPERTIES LINKER_LANGUAGE C)
#        message(STATUS "将生成可执行文件：${OUTPUT_DIR}/my_executable")
#    else()
#        message(FATAL_ERROR "启用了生成可执行文件，但主函数文件不存在！路径：${MAIN_FILE}")
#    endif()
#endif()

# 14. （可选）显式声明依赖顺序（解决跨.o依赖编译顺序问题，无则删除）
# add_dependencies(TmpTgtCalc_obj GetRealAsg_obj)
# add_dependencies(my_lib_shared TmpTgtCondCalc_obj)