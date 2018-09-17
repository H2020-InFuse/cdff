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
    image_filtering cdff_types yaml-cpp cdff_helpers opencv_imgproc opencv_core converters_opencv
)

add_library(
    image_undistortion_rectification
    ImageUndistortionRectification.cpp
)
set_target_properties(image_undistortion_rectification PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    image_undistortion_rectification
    image_filtering cdff_types yaml-cpp cdff_helpers opencv_imgproc opencv_core converters_opencv opencv_calib3d
)
