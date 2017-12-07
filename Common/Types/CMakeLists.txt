# If C folder does not exist, execute FetcherScript. 
#xma@spaceapplications.com 
if(EXISTS C)
else()
set(MAKE_CMD "${CMAKE_SOURCE_DIR}/Tools/ASNtoC/FetcherScript.sh" )
MESSAGE("COMMAND: ${MAKE_CMD}")
execute_process(COMMAND ${MAKE_CMD} RESULT_VARIABLE CMD_ERROR
      OUTPUT_FILE CMD_OUTPUT)
MESSAGE( STATUS "CMD_ERROR:" ${CMD_ERROR})
MESSAGE( STATUS "CMD_OUTPUT:" ${CMD_OUTPUT})
endif()

#compile everything from subfolders
file(GLOB_RECURSE source_list
 "*/*.c"
 "*/*.h"
)

add_library(
    cdff_types
    ${source_list}
)
install(
    TARGETS cdff_types
    DESTINATION lib
)
