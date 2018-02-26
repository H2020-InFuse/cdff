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
    image_filtering cdff_types yaml-cpp cdff_helpers opencv_imgproc opencv_core converters_opencv
)
