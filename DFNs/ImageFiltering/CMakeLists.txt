add_library(
    image_filtering
    ImageFilteringInterface.cpp
    ImageFilteringExecutor.cpp
)
set_target_properties(image_filtering PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    image_undistortion
    ImageUndistortion.cpp
)
set_target_properties(image_undistortion PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    image_undistortion
    image_filtering yaml-cpp cdff_helpers opencv_imgproc opencv_core converters_opencv
)

add_library(
    image_undistortion_rectification
    ImageUndistortionRectification.cpp
)
set_target_properties(image_undistortion_rectification PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    image_undistortion_rectification
    image_filtering yaml-cpp cdff_helpers opencv_imgproc opencv_core converters_opencv opencv_calib3d
)

add_library(cdff_dfn_background_subtraction BackgroundExtraction.cpp)
target_link_libraries(cdff_dfn_background_subtraction image_filtering cdff_helpers)

add_library(cdff_dfn_normal_vector_extraction NormalVectorExtraction.cpp)
target_link_libraries(cdff_dfn_normal_vector_extraction
    image_filtering cdff_helpers cdff_types yaml-cpp)

add_library(cdff_dfn_edge_detection EdgeDetection.cpp)
target_link_libraries(cdff_dfn_edge_detection
    image_filtering cdff_helpers cdff_types opencv_core yaml-cpp)
