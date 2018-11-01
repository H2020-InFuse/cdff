add_library(
    feature_description_2d
    FeaturesDescription2DInterface.cpp
    FeaturesDescription2DExecutor.cpp
)

add_library(
    orb_descriptor
    OrbDescriptor.cpp
)
target_link_libraries(
    orb_descriptor
    feature_description_2d cdff_types yaml-cpp cdff_helpers opencv_features2d opencv_core cdff_converters
)
