add_definitions(-DTEST_CONFIGURATIONS="${TEST_CONFIGURATIONS}")
add_definitions(-DTEST_DATA="${TEST_DATA}")

add_library(
    synthetic_generators
    SyntheticGenerators/CameraPair.cpp
)
target_link_libraries(
	synthetic_generators
	cdff_logger cdff_types cdff_converters opencv_core opencv_imgproc opencv_imgcodecs
)

add_library(
    derived_generators
    DerivedGenerators/StereoCameraCalibrator.cpp
    DerivedGenerators/SingleCameraCalibrator.cpp
)
target_link_libraries(
	derived_generators
	cdff_logger cdff_types cdff_converters opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d
)

add_executable(
    point_cloud_transformer
    SyntheticGenerators/TransformPointCloud.cpp
    SyntheticGenerators/PointCloudTransformer.cpp
)
target_compile_definitions(point_cloud_transformer PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	point_cloud_transformer
	cdff_logger cdff_types cdff_converters opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d pcl_common pcl_filters yaml-cpp ${VTK_LIBRARIES} cdff_visualizers_pcl ${PCL_IO_LIBRARIES}
)

add_executable(
    data_set_transformer
    SyntheticGenerators/TransformDataSet.cpp
)
target_link_libraries(
	data_set_transformer
	cdff_logger cdff_types cdff_converters opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d pcl_common pcl_common pcl_filters yaml-cpp ${VTK_LIBRARIES} cdff_visualizers_pcl
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
	cdff_logger cdff_types cdff_converters opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d
)

add_executable(
    match_images
    SyntheticGenerators/MatchImages.cpp
    SyntheticGenerators/ImagesMatcher.cpp
    SyntheticGenerators/ImageZooming.cpp
)
target_link_libraries(
	match_images
	cdff_logger cdff_types cdff_converters opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d
)

add_executable(
    match_multiple_images
    SyntheticGenerators/MatchMultipleImages.cpp
    SyntheticGenerators/MultipleImagesMatcher.cpp
    SyntheticGenerators/ImageZooming.cpp
)
target_link_libraries(
	match_multiple_images
	cdff_logger cdff_types cdff_converters opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d
)

add_executable(
    paint_lines
    SyntheticGenerators/PaintLines.cpp
    SyntheticGenerators/LinesPainter.cpp
    SyntheticGenerators/ImageZooming.cpp
)
target_link_libraries(
	paint_lines
	cdff_logger cdff_types cdff_converters opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d
)

add_executable(
    match_clouds
    SyntheticGenerators/MatchClouds.cpp
    SyntheticGenerators/CloudsMatcher.cpp
)
target_compile_definitions(match_clouds PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	match_clouds
	cdff_logger cdff_types cdff_converters opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d pcl_filters yaml-cpp cdff_visualizers_pcl ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_KDTREE_LIBRARIES}
)

add_executable(
    overlap_clouds
    SyntheticGenerators/OverlapClouds.cpp
    SyntheticGenerators/CloudsOverlapper.cpp
)
target_compile_definitions(match_clouds PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	overlap_clouds
	cdff_logger cdff_types cdff_converters opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d pcl_filters yaml-cpp cdff_visualizers_pcl ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_KDTREE_LIBRARIES}
)

add_executable(
    detect_outliers
    SyntheticGenerators/DetectOutliers.cpp
    SyntheticGenerators/OutliersDetector.cpp
)
target_compile_definitions(detect_outliers PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	detect_outliers
	cdff_logger cdff_types cdff_converters opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d pcl_filters yaml-cpp cdff_visualizers_pcl ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_IO_LIBRARIES}
)

add_executable(
    measures_lines
    SyntheticGenerators/MeasureLines.cpp
    SyntheticGenerators/ShapesProfiler.cpp
)
target_compile_definitions(measures_lines PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	measures_lines
	cdff_logger cdff_types cdff_converters opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d pcl_common pcl_filters yaml-cpp cdff_visualizers_pcl ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_KDTREE_LIBRARIES} ${PCL_IO_LIBRARIES}
)

add_executable(
    simulate_stereo_cloud
    SyntheticGenerators/StereoCloudSimulator.cpp
    SyntheticGenerators/SimulateStereoCloud.cpp
)
target_compile_definitions(simulate_stereo_cloud PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	simulate_stereo_cloud
	cdff_logger cdff_types cdff_converters cdff_visualizers_pcl opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d yaml-cpp ${VTK_LIBRARIES} pcl_common pcl_filters pcl_io pcl_io_ply pcl_kdtree
)

add_executable(
    generate_point_clouds
    SyntheticGenerators/GeneratePointCloud.cpp
    SyntheticGenerators/PointCloudGenerator.cpp
)
target_compile_definitions(generate_point_clouds PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	generate_point_clouds
	cdff_logger cdff_types cdff_converters opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d pcl_common pcl_filters ${PCL_SEGMENTATION_LIBRARIES} yaml-cpp ${VTK_LIBRARIES} cdff_visualizers_pcl cdff_dfn_stereo_reconstruction cdff_dfn_dfnexecutors
)
