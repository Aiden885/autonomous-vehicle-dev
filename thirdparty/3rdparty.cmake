if (3RDLOAD)
    if (ARCH MATCHES "aarch64")
        set(THIRDPARTY_DIR ${THIRDPARTY_ROOT_DIR}/aarch64)
    else()
        set(THIRDPARTY_DIR ${THIRDPARTY_ROOT_DIR}/x86)
    endif()

    if (ARCH MATCHES "aarch64")
        set(HORIZON_J5_DIR ${THIRDPARTY_DIR}/dnn)
        include_directories(${HORIZON_J5_DIR}/include)
        link_directories(${HORIZON_J5_DIR}/lib)

        set(OPENCV_DIR ${THIRDPARTY_DIR}/opencv)
        include_directories(${OPENCV_DIR}/include)
        link_directories(${OPENCV_DIR}/lib)

        set(GLOG_DIR ${THIRDPARTY_DIR}/glog)
        include_directories(${GLOG_DIR}/include)
        link_directories(${GLOG_DIR}/lib)
    else()
        set(OPENCV_DIR ${THIRDPARTY_DIR}/opencv-4.2.0)
        include_directories(${OPENCV_DIR}/include/opencv4)
        link_directories(${OPENCV_DIR}/lib)

        set(TENSORRT_DIR ${THIRDPARTY_DIR}/TensorRT-8.4.3.1)
        include_directories(${TENSORRT_DIR}/include)
        link_directories(${TENSORRT_DIR}/lib)

    endif()
    set(BOOST_DIR ${THIRDPARTY_DIR}/boost-1.72.0)
    include_directories(${BOOST_DIR}/include)
    link_directories(${BOOST_DIR}/lib)
    set(Boost_INCLUDE_DIR ${BOOST_DIR}/include)
    FILE(GLOB_RECURSE BOOST_LIBS
          ${BOOST_DIR}/lib/*.so*)    

    set(EIGEN_DIR ${THIRDPARTY_DIR}/eigen-3.3.7)
    include_directories(${EIGEN_DIR}/include/eigen3)

    set(OSQP_DIR ${THIRDPARTY_DIR}/osqp-0.6.2)
    include_directories(${OSQP_DIR}/include)
    link_directories(${OSQP_DIR}/lib)

    set(COLPACK_DIR ${THIRDPARTY_DIR}/colpack-1.0)
    include_directories(${COLPACK_DIR}/include)
    link_directories(${COLPACK_DIR}/lib)
    FILE(GLOB_RECURSE  
        ${COLPACK_DIR}/lib/*.so*)

    set(OGRE_DIR ${THIRDPARTY_DIR}/ogre-1.12.1)
    include_directories(${OGRE_DIR}/include)
    link_directories(${OGRE_DIR}/lib)
    FILE(GLOB_RECURSE OGRE_LIBS
        ${OGRE_DIR}/lib/*.so* ${OGRE_DIR}/lib/*.a ${OGRE_DIR}/lib/OGRE/*.so*)

    set(ABSEIL_DIR ${THIRDPARTY_DIR}/abseil-0623.1)
    include_directories(${ABSEIL_DIR}/include)
    link_directories(${ABSEIL_DIR}/lib)
    FILE(GLOB_RECURSE ABSEIL_LIBS
            ${ABSEIL_DIR}/lib/*.so*)

    set(QT_DIR ${THIRDPARTY_DIR}/qt-5.15.7)
    include_directories(${QT_DIR}/include)
    link_directories(${QT_DIR}/lib)

    set(ADOLC_DIR ${THIRDPARTY_DIR}/adolc-2.7.2)
    include_directories(${ADOLC_DIR}/include)
    link_directories(${ADOLC_DIR}/lib)   
    
    set(IPOPT_DIR ${THIRDPARTY_DIR}/ipopt-3.14.10)
    include_directories(${IPOPT_DIR}/include)
    link_directories(${IPOPT_DIR}/lib/*so*)
    FILE(GLOB_RECURSE IPOPT_LIBS 
            ${IPOPT_DIR}/lib/*so*)

    set(ADRSS_DIR ${THIRDPARTY_DIR}/ad-rss-1.x.x)
    include_directories(${ADRSS_DIR}/include)
    link_directories(${ADRSS_DIR}/lib/*so*)
    FILE(GLOB_RECURSE ADRSS_LIBS
            ${ADRSS_DIR}/lib/*so*)    

    set(VSOMEIP_DIR ${THIRDPARTY_DIR}/vsomeip-3.1.20.3)
    include_directories(${VSOMEIP_DIR}/include)
    link_directories(${VSOMEIP_DIR}/lib)
    FILE(GLOB_RECURSE VSOMEIP_LIBS
            ${VSOMEIP_DIR}/lib/*so*)

    set(GDAL_DIR ${THIRDPARTY_DIR}/gdal-2.2.4)
    include_directories(${GDAL_DIR}/include)
    link_directories(${GDAL_DIR}/lib)

    set(GTEST_DIR ${THIRDPARTY_DIR}/googletest-1.11.0)
    include_directories(${GTEST_DIR}/include)
    link_directories(${GTEST_DIR}/lib)

    set(GFLAGS_DIR ${THIRDPARTY_DIR}/gflags-2.2.2)
    include_directories(${GFLAGS_DIR}/include)
    link_directories(${GFLAGS_DIR}/lib)

    set(GLOG_DIR ${THIRDPARTY_DIR}/glog-0.4.0)
    include_directories(${GLOG_DIR}/include)
    link_directories(${GLOG_DIR}/lib)

    set(PROTOBUF_DIR ${THIRDPARTY_DIR}/protobuf-3.14.0)
    include_directories(${PROTOBUF_DIR}/include)
    link_directories(${PROTOBUF_DIR}/lib)

    set(YAMLCPP_DIR ${THIRDPARTY_DIR}/yamlcpp)
    include_directories(${YAMLCPP_DIR}/include)
    link_directories(${YAMLCPP_DIR}/lib)

    set(CERES_SOLOVER_DIR ${THIRDPARTY_DIR}/ceres-solver-1.14.0)
    include_directories(${CERES_SOLOVER_DIR}/include)
    link_directories(${CERES_SOLOVER_DIR}/lib)

    set(G2O_DIR ${THIRDPARTY_DIR}/g2o-20201223)
    include_directories(${G2O_DIR}/include)
    link_directories(${G2O_DIR}/lib)

    set(GTSAM_DIR ${THIRDPARTY_DIR}/gtsam-4.1.0)
    include_directories(${GTSAM_DIR}/include)
    link_directories(${GTSAM_DIR}/lib)

    set(OSQP_DIR ${THIRDPARTY_DIR}/osqp-0.6.3)
    include_directories(${OSQP_DIR}/include)
    link_directories(${OSQP_DIR}/lib)

    if(USE_ROS)
        set(ROS_DIR ${THIRDPARTY_DIR}/ros1)
        include_directories(${ROS_DIR}/xmlrpcpp)
        include_directories(${ROS_DIR}/include)
        link_directories(${ROS_DIR}/lib)
        FILE(GLOB_RECURSE ROS_LIBS
            ${ROS_DIR}/lib/*so*)
    endif()
    # if(USE_ROS2)
    #     set(ROS_DIR ${THIRDPARTY_DIR}/ros2)
    #     include_directories(${ROS_DIR}/include)
    #     link_directories(${ROS_DIR}/lib)
    #     FILE(GLOB_RECURSE ROS_LIBS
    #         ${ROS_DIR}/lib/*so*)
    # endif()

    

    set(PROTOC_BIN ${THIRDPARTY_ROOT_DIR}/x86/protobuf-3.14.0/bin/protoc)

    function(generate_proto input_dir output_dir output_cpps output_headers)
        set(proto_include_dir )
        foreach(arg IN LISTS ARGN)
            set(proto_include_dir -I ${arg} ${proto_include_dir})
        endforeach()
        message(STATUS "proto_include_dir: ${proto_include_dir}")
        execute_process(COMMAND mkdir -p ${output_dir})
        file(GLOB subobjs RELATIVE ${input_dir} ${input_dir}/*)
        set(subdirs "")
        LIST(APPEND subdirs ".")
        foreach(obj ${subobjs})
            if(IS_DIRECTORY ${input_dir}/${obj})
                LIST(APPEND subdirs ${obj})
            endif()
        endforeach()
        message(STATUS "subdirs is ${subdirs}")

        set(${output_cpps})
        set(${output_headers})
        foreach(subdir ${subdirs})
            FILE(GLOB_RECURSE proto_files
            ${input_dir}/${subdir}/*.proto)
                    
            foreach(file ${proto_files})
                get_filename_component(file_name ${file} NAME_WE)
                set(output_cpp ${output_dir}/${subdir}/${file_name}.pb.cc)
                set(output_header ${output_dir}/${subdir}/${file_name}.pb.h)
                list(APPEND ${output_cpps} ${output_cpp})
                list(APPEND ${output_headers} ${output_header})
                # set(${output_cpps} "${output_cpps} ${output_cpp}" PARENT_SCOPE)
                # set(${output_headers} "${output_headers}  ${output_header}" PARENT_SCOPE)
                add_custom_command(PRE_BUILD
                    OUTPUT ${output_header} ${output_cpp} 
                    COMMAND ${PROTOC_BIN} ${file} --cpp_out ${output_dir} -I ${input_dir}/${subdir} ${proto_include_dir}
                    DEPENDS ${PROTOC_BIN} ${file} ${output_dir}
                    VERBATIM
                )
            endforeach(file ${proto_files})
        endforeach(subdir ${subdirs})
        set(${output_cpps} ${${output_cpps}} PARENT_SCOPE)
        set(${output_headers} ${${output_headers}} PARENT_SCOPE)
    endfunction(generate_proto)
endif()
set(3RDLOAD off)

