add_library(
    model_based_visual_tracking
    ModelBasedVisualTrackingInterface.cpp
)
target_link_libraries(
    model_based_visual_tracking
)

add_library(
    dfpc_implementation_edge_model_contour_matching
    EdgeModelContourMatching.cpp
)

target_link_libraries(
    dfpc_implementation_edge_model_contour_matching
    model_based_visual_tracking cdff_types yaml-cpp cdff_helpers opencv_imgproc opencv_core opencv_imgcodecs converters_opencv cdff_pcl_visualizer dfpc_configurator DLRtracker_core
)





