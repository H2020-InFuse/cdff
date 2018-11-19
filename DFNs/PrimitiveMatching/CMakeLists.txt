add_library(
    primitive_matching
    PrimitiveMatchingInterface.cpp
)

add_library(
    hu_invariants
    HuInvariants.cpp
)
target_link_libraries(
    hu_invariants
    primitive_matching cdff_types yaml-cpp cdff_helpers opencv_imgproc opencv_core cdff_converters
)
