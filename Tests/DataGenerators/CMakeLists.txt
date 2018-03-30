add_definitions(-DTEST_CONFIGURATIONS="${TEST_CONFIGURATIONS}")
add_definitions(-DTEST_DATA="${TEST_DATA}")

add_library(
    synthetic_generators
    SyntheticGenerators/CameraPair.cpp
)
target_link_libraries(
	synthetic_generators
	cdff_logger cdff_types converters_opencv opencv_core opencv_imgproc opencv_imgcodecs 
)

add_library(
    derived_generators
    DerivedGenerators/StereoCameraCalibrator.cpp
    DerivedGenerators/SingleCameraCalibrator.cpp
)
target_link_libraries(
	derived_generators
	cdff_logger cdff_types converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d
)

add_executable(
	calibrate_stereo_camera
	DerivedGenerators/CalibrateStereoCamera.cpp
)
target_link_libraries(
	calibrate_stereo_camera
	derived_generators
)

add_executable(
	calibrate_single_camera
	DerivedGenerators/CalibrateSingleCamera.cpp
)
target_link_libraries(
	calibrate_single_camera
	derived_generators
)
