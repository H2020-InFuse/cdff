add_library(
    k_means_2d
    KMeans2DInterface.cpp
)

add_library(
    k_means
    Kmeans.cpp
)

target_link_libraries(
    k_means
    k_means_2d cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core converters_opencv cdff_opencv_visualizer
)