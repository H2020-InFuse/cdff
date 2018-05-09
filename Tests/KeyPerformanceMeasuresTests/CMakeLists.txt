#DFNs
add_executable(
    validity_harris_detector_2d
    DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp
    DFNs/FeaturesExtraction2D/SelectionTester.cpp
)
target_link_libraries(
	validity_harris_detector_2d
	cdff_helpers cdff_logger cdff_types converters_opencv harris_detector_2d feature_extraction_2d mock opencv_core opencv_imgproc opencv_highgui boost_system
)

add_executable(
    linearity_image_undistortion
    DFNs/ImageFiltering/ImageUndistortion.cpp
    DFNs/ImageFiltering/LinearityTester.cpp
)
target_link_libraries(
	linearity_image_undistortion
	cdff_helpers cdff_logger cdff_types converters_opencv image_undistortion image_filtering mock opencv_core opencv_imgproc opencv_highgui boost_system
)

add_executable(
    regularity_harris_detector_3d
    DFNs/FeaturesExtraction3D/HarrisDetector3D.cpp
    DFNs/FeaturesExtraction3D/RegularityTester.cpp
)
target_link_libraries(
	regularity_harris_detector_3d
	cdff_helpers cdff_logger cdff_types converters_opencv harris_detector_3d feature_extraction_3d mock pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters boost_system
)


#DFNs Integration
add_executable(
    correctness_harris_shot_icp
    DFNsIntegration/Odometry3D/HarrisShotIcp.cpp
    DFNsIntegration/Odometry3D/CorrectLocalizationTester.cpp
)
target_link_libraries(
	correctness_harris_shot_icp
	cdff_helpers cdff_logger cdff_types converters_opencv harris_detector_3d feature_extraction_3d shot_descriptor_3d feature_description_3d icp_3d feature_matching_3d mock pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters boost_system
)

add_executable(
    validity_orb_flann
    DFNsIntegration/Matching2D/OrbFlann.cpp
    DFNsIntegration/Matching2D/SelectionTester.cpp
)
target_link_libraries(
	validity_orb_flann
	cdff_helpers cdff_logger cdff_types converters_opencv orb_descriptor flann_matcher mock opencv_core opencv_imgproc opencv_highgui boost_system
)
