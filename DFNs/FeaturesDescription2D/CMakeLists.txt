add_library(
    feature_description_2d
    FeaturesDescription2DInterface.cpp
    FeaturesDescription2DExecutor.cpp
)
set_target_properties(feature_description_2d PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    orb_descriptor
    OrbDescriptor.cpp
)
set_target_properties(orb_descriptor PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    orb_descriptor
    feature_description_2d cdff_types yaml-cpp cdff_helpers opencv_features2d opencv_core converters_opencv
)
