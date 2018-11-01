add_library(
    voxelization
    VoxelizationInterface.cpp
    VoxelizationExecutor.cpp
)

add_library(
    octree
    Octree.cpp
)

target_link_libraries(
    octree
    voxelization cdff_types yaml-cpp cdff_helpers opencv_core cdff_converters
)
