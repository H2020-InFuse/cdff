add_library(
    depth_filtering
    DepthFilteringInterface.cpp
    DepthFilteringExecutor.cpp
)

add_library(
    filter
    Filter.cpp
)

target_link_libraries(
    filter
    depth_filtering cdff_types yaml-cpp cdff_helpers opencv_core converters_opencv
)
