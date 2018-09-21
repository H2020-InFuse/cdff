add_library(
    image_filtering
    ImageFilteringInterface.cpp
    ImageFilteringExecutor.cpp
)

add_library(
    image_undistortion
    ImageUndistortion.cpp
)
target_link_libraries(
    image_undistortion
    image_filtering yaml-cpp cdff_helpers opencv_imgproc opencv_core converters_opencv
)

add_library(
    image_undistortion_rectification
    ImageUndistortionRectification.cpp
)
target_link_libraries(
    image_undistortion_rectification
    image_filtering yaml-cpp cdff_helpers opencv_imgproc opencv_core converters_opencv opencv_calib3d
)

add_library(
        validators
        Validators.cpp
)

add_library(cdff_dfn_background_subtraction BackgroundExtraction.cpp)
target_link_libraries(cdff_dfn_background_subtraction validators image_filtering cdff_helpers)

add_library(cdff_dfn_normal_vector_extraction NormalVectorExtraction.cpp)
target_link_libraries(cdff_dfn_normal_vector_extraction
    validators image_filtering cdff_helpers cdff_types yaml-cpp)

add_library(cdff_dfn_edge_detection EdgeDetection.cpp)
target_link_libraries(cdff_dfn_edge_detection
    validators image_filtering cdff_helpers cdff_types opencv_core yaml-cpp)
