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
	mock cdff_logger harris_detector_2d converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui
)
