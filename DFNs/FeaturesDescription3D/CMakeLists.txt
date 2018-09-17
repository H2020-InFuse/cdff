add_library(
    feature_description_3d
    FeaturesDescription3DInterface.cpp
    FeaturesDescription3DExecutor.cpp
)
set_target_properties(feature_description_3d PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    shot_descriptor_3d
    ShotDescriptor3D.cpp
)
set_target_properties(shot_descriptor_3d PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    shot_descriptor_3d
    feature_description_3d cdff_types yaml-cpp cdff_helpers converters_opencv
)
