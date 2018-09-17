add_library(
    point_cloud_assembly
    PointCloudAssemblyInterface.cpp
    PointCloudAssemblyExecutor.cpp
)
set_target_properties(point_cloud_assembly PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    neighbour_point_average
    NeighbourPointAverage.cpp
)
set_target_properties(neighbour_point_average PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    neighbour_point_average
    point_cloud_assembly cdff_types yaml-cpp cdff_helpers converters_opencv
)
