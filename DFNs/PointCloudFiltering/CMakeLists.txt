add_library(
    point_cloud_filtering
    PointCloudFilteringInterface.cpp
)

add_library(
    statistical_outlier_removal
    StatisticalOutlierRemoval.cpp
)
target_link_libraries(
    statistical_outlier_removal
    point_cloud_filtering cdff_types yaml-cpp cdff_helpers cdff_converters
)
