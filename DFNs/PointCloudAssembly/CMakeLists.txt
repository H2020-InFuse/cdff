add_library(
    point_cloud_assembly
    PointCloudAssemblyInterface.cpp
)

add_library(
    neighbour_point_average
    NeighbourPointAverage.cpp
)
target_link_libraries(
    neighbour_point_average
    point_cloud_assembly cdff_types yaml-cpp cdff_helpers cdff_converters
)

add_library(
    voxel_binning
    VoxelBinning.cpp
)
target_link_libraries(
    voxel_binning
    point_cloud_assembly cdff_types yaml-cpp cdff_helpers cdff_converters
)

add_library(
    matcher_assembly
    MatcherAssembly.cpp
)
target_include_directories(
    matcher_assembly
    SYSTEM PRIVATE ${POINTMATCHER_INCLUDE_DIRS}
)
target_link_libraries(
    matcher_assembly
    point_cloud_assembly cdff_types yaml-cpp cdff_helpers cdff_converters ${POINTMATCHER_LIBRARIES}
)
