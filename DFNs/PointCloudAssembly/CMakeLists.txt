add_library(
    point_cloud_assembly
    PointCloudAssemblyInterface.cpp
    PointCloudAssemblyExecutor.cpp
)

add_library(
    neighbour_point_average
    NeighbourPointAverage.cpp
)
target_link_libraries(
    neighbour_point_average
    point_cloud_assembly cdff_types yaml-cpp cdff_helpers converters_opencv
)
