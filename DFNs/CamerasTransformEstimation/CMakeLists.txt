add_library(
    cameras_transform_estimation
    CamerasTransformEstimationInterface.cpp
    CamerasTransformEstimationExecutor.cpp
)
set_target_properties(cameras_transform_estimation PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    essential_matrix_decomposition
    EssentialMatrixDecomposition.cpp
)
set_target_properties(essential_matrix_decomposition PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    essential_matrix_decomposition
    cameras_transform_estimation cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core converters_opencv
)
