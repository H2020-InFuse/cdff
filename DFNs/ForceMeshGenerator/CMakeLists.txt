set(FORCE_MESH_GENERATOR_SOURCES "ForceMeshGeneratorInterface.cpp")
set(FORCE_MESH_GENERATOR_INCLUDE_DIRS "")
set(FORCE_MESH_GENERATOR_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(FORCE_MESH_GENERATOR_SOURCES ${FORCE_MESH_GENERATOR_SOURCES} "ThresholdForce.cpp")
	set(FORCE_MESH_GENERATOR_INCLUDE_DIRS ${FORCE_MESH_GENERATOR_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
	set(FORCE_MESH_GENERATOR_DEPENDENCIES ${FORCE_MESH_GENERATOR_DEPENDENCIES} ${OpenCV_LIBRARIES})
endif()

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
