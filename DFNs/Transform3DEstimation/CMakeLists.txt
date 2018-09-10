add_library(
    transform_3d_estimation
    Transform3DEstimationInterface.cpp
    Transform3DEstimationExecutor.cpp
)
set_target_properties(transform_3d_estimation PROPERTIES COMPILE_FLAGS -fPIC)

add_library(
    transform_3d_estimation_least_squares_minimization
    LeastSquaresMinimization.cpp
)
set_target_properties(transform_3d_estimation_least_squares_minimization PROPERTIES COMPILE_FLAGS -fPIC)
target_link_libraries(
    transform_3d_estimation_least_squares_minimization
    transform_3d_estimation cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core converters_opencv
)

add_library(
    transform_3d_estimation_ceres
    CeresEstimation.cpp
)
set_target_properties(transform_3d_estimation_ceres PROPERTIES COMPILE_FLAGS -fPIC)
target_link_libraries(
    transform_3d_estimation_ceres
    transform_3d_estimation cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core converters_opencv ${CERES_LIBRARIES}
)
