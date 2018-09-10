add_library(
    bundle_adjustment
    BundleAdjustmentInterface.cpp
    BundleAdjustmentExecutor.cpp
)
set_target_properties(bundle_adjustment PROPERTIES COMPILE_FLAGS -fPIC)

add_library(
    svd_decomposition
    SvdDecomposition.cpp
)
set_target_properties(svd_decomposition PROPERTIES COMPILE_FLAGS -fPIC)
target_link_libraries(
    svd_decomposition
    bundle_adjustment cdff_types yaml-cpp cdff_helpers opencv_core converters_opencv
)

add_library(
    ceres_adjustment
    CeresAdjustment.cpp
)
set_target_properties(ceres_adjustment PROPERTIES COMPILE_FLAGS -fPIC)
target_link_libraries(
    ceres_adjustment
    bundle_adjustment cdff_types yaml-cpp cdff_helpers cdff_logger opencv_core converters_opencv ${CERES_LIBRARIES}
)
