add_library(
    fundamental_matrix_computation
    FundamentalMatrixComputationInterface.cpp
    FundamentalMatrixComputationExecutor.cpp
)
set_target_properties(fundamental_matrix_computation PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    fundamental_matrix_ransac
    FundamentalMatrixRansac.cpp
)
set_target_properties(fundamental_matrix_ransac PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    fundamental_matrix_ransac
    fundamental_matrix_computation cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core converters_opencv
)
