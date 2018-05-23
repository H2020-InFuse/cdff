add_library(
    normal_map_2d
    NormalMap2DInterface.cpp
)

add_library(
    normals_extractor_2d
    NormalsExtractor2D.cpp
)

target_link_libraries(
    normals_extractor_2d
    normal_map_2d cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core converters_opencv cdff_opencv_visualizer
)