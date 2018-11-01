add_library(
    feature_description_3d
    FeaturesDescription3DInterface.cpp
    FeaturesDescription3DExecutor.cpp
)

add_library(
    shot_descriptor_3d
    ShotDescriptor3D.cpp
)
target_link_libraries(
    shot_descriptor_3d
    feature_description_3d cdff_types yaml-cpp cdff_helpers cdff_converters
)
