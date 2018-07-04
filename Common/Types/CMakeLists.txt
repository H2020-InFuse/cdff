# If Types/C folder does not exist we should regenerate the ASN C code.
#
# If mono is installed then we can generate them locally (GeneratorScript.sh),
# otherwise we can download them from gitlab (FetcherScript.sh)
#
# xma@spaceapplications.com, tch@spaceapplications.com

message(STATUS "Looking for ASN.1 C bindings in ${CMAKE_CURRENT_SOURCE_DIR}/C")
if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/C)
    message(STATUS "ASN.1 C bindings found. Skipping generation step")
else()
    # Use _NOT_FOUND variables instead of _FOUND variables since 0 indicates
    # that the package was found and 1 indicates that it was missing.
    execute_process(
        COMMAND dpkg -l libmono-system-core4.0-cil
        RESULT_VARIABLE MONO_CORE_NOT_FOUND
    )
    execute_process(
        COMMAND dpkg -l libmono-system-numerics4.0-cil
        RESULT_VARIABLE MONO_NUMERICS_NOT_FOUND
    )

    if(${MONO_CORE_NOT_FOUND} OR ${MONO_NUMERICS_NOT_FOUND})
        message(STATUS "Mono installation not found. Downloading ASN.1 bindings.")
        set(MAKE_CMD "${CMAKE_SOURCE_DIR}/Tools/ASNtoC/FetcherScript.sh" )
    else()
        message(STATUS "Found mono installation. Generating ASN.1 bindings locally")
        set(MAKE_CMD "${CMAKE_SOURCE_DIR}/Tools/ASNtoC/GeneratorScript.sh" )
    endif()

    MESSAGE("COMMAND: ${MAKE_CMD}")
    execute_process(COMMAND ${MAKE_CMD} RESULT_VARIABLE CMD_ERROR
          OUTPUT_FILE CMD_OUTPUT)
    MESSAGE(STATUS "CMD_ERROR:" ${CMD_ERROR})
    MESSAGE(STATUS "CMD_OUTPUT:" ${CMD_OUTPUT})

endif()

#compile everything from subfolders
file(GLOB_RECURSE source_list
 "CPP/*.cpp"
 "C/*.c"
)

add_library(
    cdff_types
    ${source_list}
)
install(
    TARGETS cdff_types
    DESTINATION lib
)
target_link_libraries(
    cdff_types cdff_logger
)
