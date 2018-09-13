add_library(
    point_cloud_transform
    PointCloudTransformInterface.cpp
    PointCloudTransformExecutor.cpp
)
set_target_properties(point_cloud_transform PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    cartesian_system_transform
    CartesianSystemTransform.cpp
)
set_target_properties(cartesian_system_transform PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    cartesian_system_transform
    point_cloud_transform cdff_types yaml-cpp cdff_helpers converters_opencv
)
