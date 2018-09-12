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
	cdff_logger cdff_types converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d pcl_common pcl_filters boost_system yaml-cpp ${VTK_LIBRARIES} boost_system cdff_pcl_visualizer
)

add_executable(
    data_set_transformer
    SyntheticGenerators/TransformDataSet.cpp
)
target_link_libraries(
	data_set_transformer
	cdff_logger cdff_types converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d pcl_common pcl_common pcl_filters boost_system yaml-cpp ${VTK_LIBRARIES} boost_system cdff_pcl_visualizer
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

add_executable(
	images_scrapper
	SyntheticGenerators/ImagesScrapper.cpp
)
target_link_libraries(
	images_scrapper
	cdff_logger
)

add_executable(
    paint_image
    SyntheticGenerators/PaintImage.cpp
    SyntheticGenerators/ImagePainter.cpp
    SyntheticGenerators/ImageZooming.cpp
)
target_link_libraries(
	paint_image
	cdff_logger cdff_types converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d
)

add_executable(
    match_images
    SyntheticGenerators/MatchImages.cpp
    SyntheticGenerators/ImagesMatcher.cpp
    SyntheticGenerators/ImageZooming.cpp
)
target_link_libraries(
	match_images
	cdff_logger cdff_types converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d
)

add_executable(
    match_multiple_images
    SyntheticGenerators/MatchMultipleImages.cpp
    SyntheticGenerators/MultipleImagesMatcher.cpp
    SyntheticGenerators/ImageZooming.cpp
)
target_link_libraries(
	match_multiple_images
	cdff_logger cdff_types converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d
)

add_executable(
    paint_lines
    SyntheticGenerators/PaintLines.cpp
    SyntheticGenerators/LinesPainter.cpp
    SyntheticGenerators/ImageZooming.cpp
)
target_link_libraries(
	paint_lines
	cdff_logger cdff_types converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d
)

add_executable(
    match_clouds
    SyntheticGenerators/MatchClouds.cpp
    SyntheticGenerators/CloudsMatcher.cpp
)
target_link_libraries(
	match_clouds
	cdff_logger cdff_types converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d pcl_common pcl_filters boost_system yaml-cpp ${VTK_LIBRARIES} boost_system cdff_pcl_visualizer
)

add_executable(
    detect_outliers
    SyntheticGenerators/DetectOutliers.cpp
    SyntheticGenerators/OutliersDetector.cpp
)
target_link_libraries(
	detect_outliers
	cdff_logger cdff_types converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d pcl_common pcl_filters boost_system yaml-cpp ${VTK_LIBRARIES} boost_system cdff_pcl_visualizer
)

add_executable(
    measures_lines
    SyntheticGenerators/MeasureLines.cpp
    SyntheticGenerators/ShapesProfiler.cpp
)
target_link_libraries(
	measures_lines
	cdff_logger cdff_types converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d pcl_common pcl_filters boost_system yaml-cpp ${VTK_LIBRARIES} boost_system cdff_pcl_visualizer
)
