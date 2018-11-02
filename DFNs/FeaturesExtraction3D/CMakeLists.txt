add_library(
    feature_extraction_3d
    FeaturesExtraction3DInterface.cpp
    FeaturesExtraction3DExecutor.cpp
)

add_library(
    harris_detector_3d
    HarrisDetector3D.cpp
)
target_link_libraries(
    harris_detector_3d
    feature_extraction_3d cdff_types yaml-cpp cdff_helpers cdff_converters ${PCL_FEATURES_LIBRARIES}
)
