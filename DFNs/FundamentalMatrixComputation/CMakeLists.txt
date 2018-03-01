add_library(
    fundamental_matrix_computation
    FundamentalMatrixComputationInterface.cpp
)

add_library(
    fundamental_matrix_ransac
    FundamentalMatrixRansac.cpp
)
target_link_libraries(
    fundamental_matrix_ransac
    fundamental_matrix_computation cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core converters_opencv
)
