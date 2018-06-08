add_library(
    bundle_adjustment
    BundleAdjustmentInterface.cpp
)

add_library(
    svd_decomposition
    SvdDecomposition.cpp
)
target_link_libraries(
    svd_decomposition
    bundle_adjustment cdff_types yaml-cpp cdff_helpers opencv_core converters_opencv
)
