add_library(
    point_cloud_transform
    PointCloudTransformInterface.cpp
    PointCloudTransformExecutor.cpp
)

add_library(
    cartesian_system_transform
    CartesianSystemTransform.cpp
)
target_link_libraries(
    cartesian_system_transform
    point_cloud_transform cdff_types yaml-cpp cdff_helpers cdff_converters
)
