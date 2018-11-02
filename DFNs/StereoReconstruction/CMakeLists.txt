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
    stereoReconstruction cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core cdff_converters cdff_opencv_visualizer pcl_common pcl_filters
)

add_library(
    hirschmuller_disparity_mapping
    HirschmullerDisparityMapping.cpp
)
target_link_libraries(
    hirschmuller_disparity_mapping
    stereoReconstruction cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core cdff_converters cdff_opencv_visualizer pcl_common pcl_filters
)

add_library(
    scanline_optimization
    ScanlineOptimization.cpp
)
target_link_libraries(
    scanline_optimization
    stereoReconstruction cdff_types yaml-cpp cdff_helpers cdff_converters cdff_pcl_visualizer pcl_common pcl_filters
)
