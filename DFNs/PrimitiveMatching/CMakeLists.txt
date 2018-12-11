set(PRIMITIVE_MATCHING_SOURCES "PrimitiveMatchingInterface.cpp")
set(PRIMITIVE_MATCHING_INCLUDE_DIRS "")
set(PRIMITIVE_MATCHING_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(PRIMITIVE_MATCHING_SOURCES ${PRIMITIVE_MATCHING_SOURCES} "HuInvariants.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(PRIMITIVE_MATCHING_DEPENDENCIES ${PRIMITIVE_MATCHING_DEPENDENCIES} ${OpenCV_LIBS})
endif()

add_library(
    cdff_dfn_primitive_matching
    ${PRIMITIVE_MATCHING_SOURCES}
)
target_include_directories(
    cdff_dfn_primitive_matching
    SYSTEM PRIVATE ${PRIMITIVE_MATCHING_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_primitive_matching
    ${PRIMITIVE_MATCHING_DEPENDENCIES}
)
