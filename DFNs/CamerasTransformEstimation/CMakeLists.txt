add_library(
    cameras_transform_estimation
    CamerasTransformEstimationInterface.cpp
)

add_library(
    essential_matrix_ransac
    EssentialMatrixRansac.cpp
)
target_link_libraries(
    essential_matrix_ransac
    cameras_transform_estimation cdff_types yaml-cpp opencv_calib3d cdff_helpers
)
