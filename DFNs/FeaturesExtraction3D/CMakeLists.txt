add_library(
    feature_extraction_3d
    FeaturesExtraction3DInterface.cpp
    FeaturesExtraction3DExecutor.cpp
)
set_target_properties(feature_extraction_3d PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    harris_detector_3d
    HarrisDetector3D.cpp
)
set_target_properties(harris_detector_3d PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    harris_detector_3d
    feature_extraction_3d cdff_types yaml-cpp cdff_helpers converters_opencv
)
