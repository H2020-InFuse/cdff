add_library(
    feature_extraction_2d
    FeaturesExtraction2DInterface.cpp
)

add_library(
    harris_detector_2d
    HarrisDetector2D.cpp
)
target_link_libraries(
    harris_detector_2d
    feature_extraction_2d cdff_types yaml-cpp cdff_helpers opencv_core cdff_converters opencv_imgproc
)

add_library(
    orb_detector_descriptor
    OrbDetectorDescriptor.cpp
)
target_link_libraries(
    orb_detector_descriptor
    feature_extraction_2d cdff_types yaml-cpp opencv_features2d cdff_helpers opencv_core cdff_converters
)
