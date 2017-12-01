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
    feature_extraction_2d cdff_types xml_helper
)
