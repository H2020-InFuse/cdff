add_library(
    stereoReconstruction
    StereoReconstructionInterface.cpp
    StereoReconstructionExecutor.cpp
)
set_target_properties(stereoReconstruction PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    disparity_mapping
    DisparityMapping.cpp
)
set_target_properties(disparity_mapping PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    disparity_mapping
    stereoReconstruction cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core converters_opencv cdff_opencv_visualizer pcl_common pcl_filters
)

add_library(
    hirschmuller_disparity_mapping
    HirschmullerDisparityMapping.cpp
)
set_target_properties(hirschmuller_disparity_mapping PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    hirschmuller_disparity_mapping
    stereoReconstruction cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core converters_opencv cdff_opencv_visualizer pcl_common pcl_filters
)

add_library(
    scanline_optimization
    ScanlineOptimization.cpp
)
set_target_properties(scanline_optimization PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    scanline_optimization
    stereoReconstruction cdff_types yaml-cpp cdff_helpers converters_opencv cdff_pcl_visualizer pcl_common pcl_filters
)
