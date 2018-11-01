add_library(
    force_mesh_generator
    ForceMeshGeneratorInterface.cpp
    ForceMeshGeneratorExecutor.cpp
)
target_link_libraries(force_mesh_generator cdff_validators)

add_library(
    threshold_force
    ThresholdForce.cpp
)

target_link_libraries(
    threshold_force
    force_mesh_generator yaml-cpp cdff_helpers opencv_core cdff_converters
)
