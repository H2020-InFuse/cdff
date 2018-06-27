# If Types/C folder does not exist we should regenerate the ASN C code.
#
# If mono is installed then we can generate them locally (GeneratorScript.sh),
# otherwise we can download them from gitlab (FetcherScript.sh)
#
# xma@spaceapplications.com, tch@spaceapplications.com

message("test if exist = ${CMAKE_CURRENT_SOURCE_DIR}/C")
if(NOT IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/C)
    find_program(MONO_BINARY mono)
    if(MONO_BINARY)
        set(MAKE_CMD "${CMAKE_SOURCE_DIR}/Tools/ASNtoC/GeneratorScript.sh" )
    else()
        set(MAKE_CMD "${CMAKE_SOURCE_DIR}/Tools/ASNtoC/FetcherScript.sh" )
    endif()

    MESSAGE("COMMAND: ${MAKE_CMD}")
    execute_process(COMMAND ${MAKE_CMD} RESULT_VARIABLE CMD_ERROR
          OUTPUT_FILE CMD_OUTPUT)
    MESSAGE( STATUS "CMD_ERROR:" ${CMD_ERROR})
    MESSAGE( STATUS "CMD_OUTPUT:" ${CMD_OUTPUT})
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
