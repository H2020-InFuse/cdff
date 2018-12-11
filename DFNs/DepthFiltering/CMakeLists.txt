set(DEPTH_FILTERING_SOURCES "DepthFilteringInterface.cpp")
set(DEPTH_FILTERING_INCLUDE_DIRS "")
set(DEPTH_FILTERING_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(DEPTH_FILTERING_SOURCES ${DEPTH_FILTERING_SOURCES} "ConvolutionFilter.cpp")
	set(DEPTH_FILTERING_INCLUDE_DIRS ${DEPTH_FILTERING_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
	set(DEPTH_FILTERING_DEPENDENCIES ${DEPTH_FILTERING_DEPENDENCIES} ${OpenCV_LIBRARIES})
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
