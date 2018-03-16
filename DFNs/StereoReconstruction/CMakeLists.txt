add_library(
    stereoReconstruction
    StereoReconstructionInterface.cpp
)

add_library(
    disparity_mapping
    DisparityMapping.cpp
)
target_link_libraries(
    disparity_mapping
     stereoReconstruction cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core converters_opencv cdff_opencv_visualizer
)

add_library(
    hirschmuller_disparity_mapping
    HirschmullerDisparityMapping.cpp
)
target_link_libraries(
    hirschmuller_disparity_mapping
     stereoReconstruction cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core converters_opencv cdff_opencv_visualizer
)

add_library(
    scanline_optimization
    ScanlineOptimization.cpp
)
target_link_libraries(
    scanline_optimization
    stereoReconstruction cdff_types yaml-cpp cdff_helpers converters_opencv cdff_pcl_visualizer
)
