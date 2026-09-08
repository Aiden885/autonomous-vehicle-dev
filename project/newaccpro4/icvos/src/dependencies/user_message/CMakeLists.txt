cmake_minimum_required(VERSION 3.16.1)
project(user_message)

set(proto_dir ${CMAKE_CURRENT_SOURCE_DIR})

set(PROTO_CPPS "")
set(PROTO_HEADERS "")
set(PROTO_INCLUDE_DIR 
        ${COMMON_MSG_PROTO_DIR}/common 
        ${COMMON_MSG_PROTO_DIR}/canbus 
        ${COMMON_MSG_PROTO_DIR}/control 
        ${COMMON_MSG_PROTO_DIR}/epsilon
        ${COMMON_MSG_PROTO_DIR}/localization 
        ${COMMON_MSG_PROTO_DIR}/map
        ${COMMON_MSG_PROTO_DIR}/planning
        ${COMMON_MSG_PROTO_DIR}/vehicle_state
        ${COMMON_MSG_PROTO_DIR}/hmi
        ${COMMON_MSG_PROTO_DIR}/hdmap
        ${COMMON_MSG_PROTO_DIR}/prediction)


file(GLOB all_items RELATIVE ${proto_dir} ${proto_dir}/*)
set(SUB_DIRS "")
foreach(item ${all_items})
    if(IS_DIRECTORY ${proto_dir}/${item})
        file(GLOB proto_files ${proto_dir}/${item}/*.proto)
        if(proto_files)
            list(APPEND SUB_DIRS ${item})
        endif()
    endif()
endforeach()

message(STATUS "Found proto subdirectories: ${SUB_DIRS}")

file(GLOB root_proto_files ${proto_dir}/*.proto)
if(root_proto_files)
    set(root_proto_cpps "")
    set(root_proto_headers "")
    generate_proto(${proto_dir} ${PROTOCPP_DIR} root_proto_cpps root_proto_headers ${PROTO_INCLUDE_DIR})
    list(APPEND PROTO_CPPS ${root_proto_cpps})
    list(APPEND PROTO_HEADERS ${root_proto_headers})
endif()

foreach(sub_dir ${SUB_DIRS})
    set(sub_proto_cpps "")
    set(sub_proto_headers "")
    generate_proto(${proto_dir}/${sub_dir} ${PROTOCPP_DIR} sub_proto_cpps sub_proto_headers ${PROTO_INCLUDE_DIR}
    )
    list(APPEND PROTO_CPPS ${sub_proto_cpps})
    list(APPEND PROTO_HEADERS ${sub_proto_headers})
endforeach()

if (PROTO_CPPS)
    add_library(${PROJECT_NAME}_msg_pb SHARED ${PROTO_CPPS})
    target_link_libraries(${PROJECT_NAME}_msg_pb protobuf common_proto_msg_pb canbus_msg_pb control_msg_pb epsilon_msg_pb localization_msg_pb map_msg_pb planning_msg_pb vehicle_state_msg_pb hmi_msg_pb hdmap_msg_pb prediction_msg_pb)
    
    install(TARGETS ${PROJECT_NAME}_msg_pb
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
endif()
