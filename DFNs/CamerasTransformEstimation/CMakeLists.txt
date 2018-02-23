add_library(
    cameras_transform_estimation
    CamerasTransformEstimationInterface.cpp
)

add_library(
    essential_matrix_decomposition
    EssentialMatrixDecomposition.cpp
)
target_link_libraries(
    essential_matrix_decomposition
    cameras_transform_estimation cdff_types yaml-cpp opencv_calib3d cdff_helpers
)
