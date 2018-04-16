add_library(
    sobel_derivative
    SobelDerivativeInterface.cpp
)

add_library(
    sobel_scharr
    SobelScharr.cpp
)
target_link_libraries(
    sobel_scharr
    sobel_derivative cdff_types yaml-cpp cdff_helpers opencv_imgproc opencv_core converters_opencv
)

target_link_libraries(
    sobel_derivative cdff_types yaml-cpp cdff_helpers opencv_imgproc opencv_core converters_opencv opencv_calib3d
)
