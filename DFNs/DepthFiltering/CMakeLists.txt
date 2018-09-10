add_library(
    depth_filtering
    DepthFilteringInterface.cpp
    DepthFilteringExecutor.cpp
)
set_target_properties(depth_filtering PROPERTIES COMPILE_FLAGS -fPIC)

add_library(
    convolution_filter
    ConvolutionFilter.cpp
)
set_target_properties(convolution_filter PROPERTIES COMPILE_FLAGS -fPIC)

target_link_libraries(
    convolution_filter
    depth_filtering cdff_types yaml-cpp cdff_helpers opencv_core converters_opencv
)
