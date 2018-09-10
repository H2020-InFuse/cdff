add_library(
    feature_matching_3d
    FeaturesMatching3DInterface.cpp
    FeaturesMatching3DExecutor.cpp
)
set_target_properties(feature_matching_3d PROPERTIES COMPILE_FLAGS -fPIC)

add_library(
    ransac_3d
    Ransac3D.cpp
)
set_target_properties(ransac_3d PROPERTIES COMPILE_FLAGS -fPIC)
target_link_libraries(
    ransac_3d
    feature_matching_3d cdff_types yaml-cpp flann_cpp cdff_helpers
)

add_library(
    icp_3d
    Icp3D.cpp
)
set_target_properties(icp_3d PROPERTIES COMPILE_FLAGS -fPIC)
target_link_libraries(
    icp_3d
    feature_matching_3d cdff_types yaml-cpp cdff_helpers
)
