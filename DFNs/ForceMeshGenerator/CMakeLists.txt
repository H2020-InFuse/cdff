set(FORCE_MESH_GENERATOR_SOURCES "ForceMeshGeneratorInterface.cpp" "ThresholdForce.cpp")
set(FORCE_MESH_GENERATOR_INCLUDE_DIRS "")
set(FORCE_MESH_GENERATOR_DEPENDENCIES "cdff_helpers" "cdff_converters")

add_library(
    cdff_dfn_force_mesh_generator
    ${FORCE_MESH_GENERATOR_SOURCES}
)
target_include_directories(
    cdff_dfn_force_mesh_generator
    SYSTEM PRIVATE ${FORCE_MESH_GENERATOR_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_force_mesh_generator
    ${FORCE_MESH_GENERATOR_DEPENDENCIES}
)
