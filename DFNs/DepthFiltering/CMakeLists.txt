set(DEPTH_FILTERING_SOURCES "DepthFilteringInterface.cpp")
set(DEPTH_FILTERING_INCLUDE_DIRS "")
set(DEPTH_FILTERING_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(DEPTH_FILTERING_SOURCES ${DEPTH_FILTERING_SOURCES} "ConvolutionFilter.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(DEPTH_FILTERING_DEPENDENCIES ${DEPTH_FILTERING_DEPENDENCIES} opencv_core opencv_videostab)
endif()

add_library(
    cdff_dfn_depth_filtering
    ${DEPTH_FILTERING_SOURCES}
)
target_include_directories(
    cdff_dfn_depth_filtering
    SYSTEM PRIVATE ${DEPTH_FILTERING_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_depth_filtering
    ${DEPTH_FILTERING_DEPENDENCIES}
)
