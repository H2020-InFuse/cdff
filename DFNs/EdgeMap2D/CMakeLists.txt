add_library(
    edge_map_2d
    EdgeMap2DInterface.cpp
)

add_library(
    canny_edge_extractor_2d
    CannyEdgeExtractor2D.cpp
)

target_link_libraries(
    canny_edge_extractor_2d
    edge_map_2d cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core converters_opencv cdff_opencv_visualizer
)