get_filename_component(APA_VEHICLE_MODULE_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

function(apa_generate_vehicle_profile output_dir)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    set(profile "${APA_VEHICLE_MODULE_ROOT}/conf/apa_vehicle_profile.json")
    set(generator "${APA_VEHICLE_MODULE_ROOT}/tools/generate_vehicle_profile.py")
    set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/apa_vehicle_profile")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${profile}" "${generator}")
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" "${generator}" --profile "${profile}"
                --output "${generated_dir}/apaVehicleProfileGenerated.h"
        RESULT_VARIABLE result
        ERROR_VARIABLE error)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Cannot generate APA vehicle profile: ${error}")
    endif()
    set(${output_dir} "${generated_dir}" PARENT_SCOPE)
endfunction()
