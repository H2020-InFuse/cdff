add_library(
    edge_detection
    EdgeDetectionInterface.cpp
)

add_library(
    canny_detector
    CannyDetector.cpp
)
target_link_libraries(
    canny_detector
    edge_detection cdff_types yaml-cpp cdff_helpers opencv_imgproc opencv_core converters_opencv
)

target_link_libraries(
    edge_detection cdff_types yaml-cpp cdff_helpers opencv_imgproc opencv_core converters_opencv opencv_calib3d
)
