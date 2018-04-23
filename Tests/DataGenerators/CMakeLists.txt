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
    point_cloud_transformer
    SyntheticGenerators/TransformPointCloud.cpp
    SyntheticGenerators/PointCloudTransformer.cpp
)
target_link_libraries(
	point_cloud_transformer
	cdff_logger cdff_types converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo boost_system yaml-cpp ${VTK_LIBRARIES} boost_system cdff_pcl_visualizer
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
