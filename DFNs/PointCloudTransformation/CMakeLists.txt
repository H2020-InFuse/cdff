set(POINT_CLOUD_TRANSFORM_SOURCES "PointCloudTransformationInterface.cpp")
set(POINT_CLOUD_TRANSFORM_INCLUDE_DIRS "")
set(POINT_CLOUD_TRANSFORM_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(PCL_FOUND)
	set(POINT_CLOUD_TRANSFORM_SOURCES ${POINT_CLOUD_TRANSFORM_SOURCES} "CartesianSystemTransform.cpp")
	set(POINT_CLOUD_TRANSFORM_INCLUDE_DIRS ${POINT_CLOUD_TRANSFORM_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
	set(POINT_CLOUD_TRANSFORM_DEPENDENCIES ${POINT_CLOUD_TRANSFORM_DEPENDENCIES} ${PCL_COMMON_LIBRARIES} ${PCL_KDTREE_LIBRARIES} ${PCL_SEARCH_LIBRARIES})
endif()

add_library(
    cdff_dfn_point_cloud_transformation
    ${POINT_CLOUD_TRANSFORM_SOURCES}
)
target_include_directories(
    cdff_dfn_point_cloud_transformation
    SYSTEM PRIVATE ${POINT_CLOUD_TRANSFORM_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_point_cloud_transformation
    ${POINT_CLOUD_TRANSFORM_DEPENDENCIES}
)
