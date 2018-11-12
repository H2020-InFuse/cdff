add_library(
    image_filtering
    ImageFilteringInterface.cpp
)

add_library(
    image_undistortion
    ImageUndistortion.cpp
)
target_link_libraries(
    image_undistortion
    image_filtering yaml-cpp cdff_helpers opencv_imgproc opencv_core cdff_converters
)

add_library(
    image_undistortion_rectification
    ImageUndistortionRectification.cpp
)
target_link_libraries(
    image_undistortion_rectification
    image_filtering yaml-cpp cdff_helpers opencv_imgproc opencv_core cdff_converters opencv_calib3d
)

add_library(cdff_dfn_background_subtraction BackgroundExtraction.cpp)
target_link_libraries(cdff_dfn_background_subtraction cdff_validators image_filtering cdff_helpers)

add_library(cdff_dfn_normal_vector_extraction NormalVectorExtraction.cpp)
target_link_libraries(cdff_dfn_normal_vector_extraction
        cdff_validators image_filtering cdff_helpers yaml-cpp)

add_library(cdff_dfn_kmeans KMeansClustering.cpp)
target_link_libraries(cdff_dfn_kmeans image_filtering cdff_helpers cdff_validators)

add_library(cdff_dfn_edge_detection EdgeDetection.cpp)
target_link_libraries(cdff_dfn_edge_detection
        cdff_validators image_filtering cdff_helpers opencv_core yaml-cpp)
