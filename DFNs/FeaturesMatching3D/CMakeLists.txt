add_library(
    feature_matching_3d
    FeaturesMatching3DInterface.cpp
)

add_library(
    ransac_3d
    Ransac3D.cpp
)
target_link_libraries(
    ransac_3d
    feature_matching_3d cdff_types yaml-cpp flann_cpp cdff_helpers
)

add_library(
    icp_3d
    Icp3D.cpp
)
target_link_libraries(
    icp_3d
    feature_matching_3d cdff_types yaml-cpp flann_cpp cdff_helpers
)
