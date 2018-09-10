add_library(
    feature_extraction_2d
    FeaturesExtraction2DInterface.cpp
    FeaturesExtraction2DExecutor.cpp
)
set_target_properties(feature_extraction_2d PROPERTIES COMPILE_FLAGS -fPIC)

add_library(
    harris_detector_2d
    HarrisDetector2D.cpp
)
set_target_properties(harris_detector_2d PROPERTIES COMPILE_FLAGS -fPIC)
target_link_libraries(
    harris_detector_2d
    feature_extraction_2d cdff_types yaml-cpp cdff_helpers opencv_core converters_opencv
)

add_library(
    orb_detector_descriptor
    OrbDetectorDescriptor.cpp
)
set_target_properties(orb_detector_descriptor PROPERTIES COMPILE_FLAGS -fPIC)
target_link_libraries(
    orb_detector_descriptor
    feature_extraction_2d cdff_types yaml-cpp opencv_features2d cdff_helpers opencv_core converters_opencv
)
