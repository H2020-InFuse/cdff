add_library(
    bundle_adjustment
    BundleAdjustmentInterface.cpp
    BundleAdjustmentExecutor.cpp
)

add_library(
    svd_decomposition
    SvdDecomposition.cpp
)
target_link_libraries(
    svd_decomposition
    bundle_adjustment cdff_types yaml-cpp cdff_helpers opencv_core converters_opencv
)

add_library(
    ceres_adjustment
    CeresAdjustment.cpp
)
target_link_libraries(
    ceres_adjustment
    bundle_adjustment cdff_types yaml-cpp cdff_helpers cdff_logger opencv_core converters_opencv ${CERES_LIBRARIES}
)
