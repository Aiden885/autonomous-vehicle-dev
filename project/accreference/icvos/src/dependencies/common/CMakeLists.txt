cmake_minimum_required(VERSION 3.16.1)

add_subdirectory(hdmap)
add_subdirectory(coordinate)

if (${RUNTIME_OS} STREQUAL ICVOS)
    install(DIRECTORY ${ICVOS_DIR}/image 
                    DESTINATION ${CMAKE_INSTALL_PREFIX}
                    FILE_PERMISSIONS WORLD_EXECUTE WORLD_READ OWNER_EXECUTE OWNER_READ GROUP_READ GROUP_EXECUTE)
    install(DIRECTORY ${ICVOS_DIR}/sdk/sensor_module/lib/
                    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
                    FILE_PERMISSIONS WORLD_EXECUTE WORLD_READ OWNER_EXECUTE OWNER_READ GROUP_READ GROUP_EXECUTE)
#    if (${ENABLE_MDC_610})
#      install(DIRECTORY ${ICVOS_DIR}/sdk/rslidar_preprocess_node/lib/
#                    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#                    FILE_PERMISSIONS WORLD_EXECUTE WORLD_READ OWNER_EXECUTE OWNER_READ GROUP_READ GROUP_EXECUTE)                    
#    endif()                    
endif()

if(EXISTS ${COMMON_DIR}/debug_tools/${TARGET_PLATFORM}/${WITH_ROS_PATH}/)
        install(DIRECTORY debug_tools/${TARGET_PLATFORM}/${WITH_ROS_PATH}/ 
                DESTINATION ${CMAKE_INSTALL_PREFIX}/debug_tools
                FILE_PERMISSIONS WORLD_EXECUTE WORLD_READ OWNER_EXECUTE OWNER_READ GROUP_READ GROUP_EXECUTE
        )
endif()

if(EXISTS ${EXTENSION_DIR}/common_lib/)
        install(DIRECTORY ${EXTENSION_DIR}/common_lib/
                DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
endif()

if(EXISTS ${EXTENSION_DIR}/scripts/)
        install(DIRECTORY ${EXTENSION_DIR}/scripts/
                DESTINATION ${CMAKE_INSTALL_PREFIX}/scripts)
endif()

if(EXISTS ${EXTENSION_DIR}/bin/)
        install(DIRECTORY ${EXTENSION_DIR}/bin/
                DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
                FILE_PERMISSIONS WORLD_EXECUTE WORLD_READ OWNER_EXECUTE OWNER_READ GROUP_READ GROUP_EXECUTE)
endif()

if(EXISTS ${EXTENSION_DIR}/conf/global_conf/)
        install(DIRECTORY ${EXTENSION_DIR}/conf/global_conf/
                DESTINATION ${CMAKE_INSTALL_PREFIX}/conf/global_conf)
endif()