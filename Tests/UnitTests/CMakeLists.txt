add_definitions(-DTEST_CONFIGURATIONS="${TEST_CONFIGURATIONS}")
add_definitions(-DTEST_DATA="${TEST_DATA}")

add_executable(
    StubFeaturesExtraction2DTest
    DFNs/FeaturesExtraction2D/StubFeaturesExtraction2D.cpp
)
target_link_libraries(
	StubFeaturesExtraction2DTest
	StubFeaturesExtraction2D opencv_core opencv_imgproc opencv_imgcodecs
)
add_test(StubFeaturesExtraction2DTest StubFeaturesExtraction2DTest)

add_executable(
    HarrisDetector2DTest
    DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp
)
target_link_libraries(
	HarrisDetector2DTest
	mock cdff_logger harris_detector_2d converters_opencv opencv_core opencv_imgproc opencv_imgcodecs
)
add_test(HarrisDetector2DTest HarrisDetector2DTest)

add_executable(
    MatImageTypeConvertersTest
    Common/Converters/MatImageTypeConvertersTest.cpp
)
target_link_libraries(
	MatImageTypeConvertersTest converters_opencv cdff_logger opencv_core opencv_imgproc opencv_imgcodecs
)
add_test(MatImageTypeConvertersTest MatImageTypeConvertersTest)

add_executable(
    MatVisualPointFeatureVector2DConvertersTest
    Common/Converters/MatVisualPointFeatureVector2DConvertersTest.cpp
)
target_link_libraries(
	MatVisualPointFeatureVector2DConvertersTest converters_opencv cdff_logger cdff_types opencv_core opencv_imgproc opencv_imgcodecs
)
add_test(MatVisualPointFeatureVector2DConvertersTest
         MatVisualPointFeatureVector2DConvertersTest)
