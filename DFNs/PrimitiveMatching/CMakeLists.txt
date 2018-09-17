add_library(
    primitive_matching
    PrimitiveMatchingInterface.cpp
    PrimitiveMatchingExecutor.cpp
)
set_target_properties(primitive_matching PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    hu_invariants
    HuInvariants.cpp
)
set_target_properties(hu_invariants PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    hu_invariants
    primitive_matching cdff_types yaml-cpp cdff_helpers opencv_imgproc opencv_core converters_opencv
)
