add_library(
    force_mesh_generator
    ForceMeshGeneratorInterface.cpp
    ForceMeshGeneratorExecutor.cpp
)

add_library(
    threshold_force
    ThresholdForce.cpp
)

target_link_libraries(
    threshold_force
    force_mesh_generator cdff_types yaml-cpp cdff_helpers opencv_core converters_opencv
)
