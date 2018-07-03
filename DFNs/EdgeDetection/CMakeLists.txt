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
    edge_detection cdff_types yaml-cpp cdff_helpers opencv_imgproc opencv_core opencv_imgcodecs converters_opencv
)

add_library(
    sobel_derivative
    SobelDerivative.cpp
)
target_link_libraries(
    sobel_derivative
    edge_detection cdff_types yaml-cpp cdff_helpers opencv_imgproc opencv_core opencv_imgcodecs converters_opencv opencv_calib3d
)
