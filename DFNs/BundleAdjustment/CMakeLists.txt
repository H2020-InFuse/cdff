add_library(
    bundle_adjustment
    BundleAdjustmentInterface.cpp
)

add_library(
    udv_decomposition
    UdvDecomposition.cpp
)
target_link_libraries(
    udv_decomposition
    bundle_adjustment cdff_types yaml-cpp cdff_helpers opencv_core converters_opencv
)
