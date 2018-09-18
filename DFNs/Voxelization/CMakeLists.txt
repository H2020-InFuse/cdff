add_library(
    voxelization
    VoxelizationInterface.cpp
    VoxelizationExecutor.cpp
)
set_target_properties(voxelization PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    octree
    Octree.cpp
)
set_target_properties(octree PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

target_link_libraries(
    octree
    voxelization cdff_types yaml-cpp cdff_helpers opencv_core converters_opencv
)
