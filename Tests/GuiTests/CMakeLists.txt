add_executable(
    HarrisDetector2DGuiTest
    DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)

set_target_properties(HarrisDetector2DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
    HarrisDetector2DGuiTest
    cdff_logger harris_detector_2d converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui
)

add_executable(
    OrbDetectorDescriptorGuiTest
    DFNs/FeaturesExtraction2D/OrbDetectorDescriptor.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(OrbDetectorDescriptorGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
    OrbDetectorDescriptorGuiTest
    cdff_logger orb_detector_descriptor converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui
)
