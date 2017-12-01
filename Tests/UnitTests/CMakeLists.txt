add_executable(
    StubFeaturesExtraction2DTest
    DFNs/FeaturesExtraction2D/StubFeaturesExtraction2D.cpp
)
set_target_properties(
    StubFeaturesExtraction2DTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	StubFeaturesExtraction2DTest
	StubFeaturesExtraction2D opencv_core opencv_imgproc opencv_imgcodecs
)

add_executable(
    HarrisDetector2DTest
    DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp
)
set_target_properties(HarrisDetector2DTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	HarrisDetector2DTest
	mock cdff_logger harris_detector_2d converters_opencv opencv_core opencv_imgproc opencv_imgcodecs
)

add_executable(
    MatImageTypeConvertersTest
    Common/Converters/MatImageTypeConvertersTest.cpp
)
set_target_properties(MatImageTypeConvertersTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	MatImageTypeConvertersTest converters_opencv cdff_logger opencv_core opencv_imgproc opencv_imgcodecs
)

add_executable(
    MatVisualPointFeatureVector2DConvertersTest
    Common/Converters/MatVisualPointFeatureVector2DConvertersTest.cpp
)
set_target_properties(MatVisualPointFeatureVector2DConvertersTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	MatVisualPointFeatureVector2DConvertersTest converters_opencv cdff_logger cdff_types opencv_core opencv_imgproc opencv_imgcodecs
)
