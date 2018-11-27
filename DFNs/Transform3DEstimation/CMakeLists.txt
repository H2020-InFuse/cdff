add_library(
    transform_3d_estimation
    Transform3DEstimationInterface.cpp
)

add_library(
    transform_3d_estimation_least_squares_minimization
    LeastSquaresMinimization.cpp
)
target_link_libraries(
    transform_3d_estimation_least_squares_minimization
    transform_3d_estimation cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core cdff_converters
)

add_library(
    transform_3d_estimation_ceres
    CeresEstimation.cpp
)
target_link_libraries(
    transform_3d_estimation_ceres
    transform_3d_estimation cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core cdff_converters ${CERES_LIBRARIES}
)
