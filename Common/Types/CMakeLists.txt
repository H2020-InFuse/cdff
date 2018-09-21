# If Types/C folder does not exist we should regenerate the ASN C code.
#
# If mono is installed then we can generate them locally (GeneratorScript.sh),
# otherwise we can download them from gitlab (FetcherScript.sh)
#
# xma@spaceapplications.com, tch@spaceapplications.com

option(COMPILE_ASN1 "Enable local compilation of the ASN1 types" On)

message(STATUS "Looking for compiled ASN.1 types in ${CMAKE_CURRENT_SOURCE_DIR}/C ...")
if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/C)
    message(STATUS "Compiled ASN.1 types found. ")
else()
    if(COMPILE_ASN1)
        message(STATUS "Compiling ASN.1 types locally ...")
        set(MAKE_CMD "${CMAKE_SOURCE_DIR}/Tools/ASNtoC/GeneratorScript.sh" )
    else()
        message(STATUS "Downloading compiled ASN.1 types ...")
        set(MAKE_CMD "${CMAKE_SOURCE_DIR}/Tools/ASNtoC/FetcherScript.sh" )
    endif()

    MESSAGE("ASN.1 Fetch/Compile COMMAND: ${MAKE_CMD}")
    execute_process(COMMAND ${MAKE_CMD}
        RESULT_VARIABLE CMD_ERROR
        OUTPUT_FILE CMD_OUTPUT)
    MESSAGE(STATUS "ASN.1 Fetch/Compile Exit Code: " ${CMD_ERROR})
    MESSAGE(STATUS "ASN.1 Fetch/Compile Output: " ${CMD_OUTPUT})

    if(NOT ${CMD_ERROR} EQUAL 0)
        message(FATAL_ERROR "ASN.1 compilation or download failed.")
    endif()

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
