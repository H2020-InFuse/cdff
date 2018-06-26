add_executable(
    harris_icp_cc
    FeaturesRegistration/HarrisIcpCC.cpp
    FeaturesRegistration/CorrectLocalizationTester.cpp
)
target_link_libraries(
	harris_icp_cc
	cdff_helpers cdff_logger cdff_types converters_opencv harris_detector_3d registration_3d registration_icp_cc opencv_core opencv_imgproc opencv_highgui boost_system pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters boost_system
)

add_executable(
    ransac_essential_triangulation_test
    FeaturesReconstruction/EssentialMatrixDecompositionTriangulation.cpp
    FeaturesReconstruction/ReconstructionTester.cpp
)
target_link_libraries(
	ransac_essential_triangulation_test
	cdff_helpers cdff_logger cdff_types converters_opencv fundamental_matrix_ransac essential_matrix_decomposition triangulation mock opencv_core opencv_imgproc opencv_highgui boost_system pcl_common
)

add_executable(
    validity_orb_flann
    Matching2D/OrbFlann.cpp
    Matching2D/SelectionTester.cpp
)
target_link_libraries(
	validity_orb_flann
	cdff_helpers cdff_logger cdff_types converters_opencv orb_descriptor flann_matcher mock opencv_core opencv_imgproc opencv_highgui boost_system
)

add_executable(
    validity_shot_icp
    Matching3D/ShotIcp.cpp
    Matching3D/SelectionTester.cpp
)
target_link_libraries(
	validity_shot_icp
	cdff_helpers cdff_logger cdff_types mock converters_opencv shot_descriptor_3d feature_description_3d icp_3d feature_matching_3d  pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters boost_system
)

add_executable(
    validity_shot_ransac
    Matching3D/ShotRansac.cpp
    Matching3D/SelectionTester.cpp
)
target_link_libraries(
	validity_shot_ransac
	cdff_helpers cdff_logger cdff_types mock converters_opencv shot_descriptor_3d feature_description_3d ransac_3d feature_matching_3d pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters boost_system
)

add_executable(
    correctness_harris_shot_icp
    Odometry3D/HarrisShotIcp.cpp
    Odometry3D/CorrectLocalizationTester.cpp
)
target_link_libraries(
	correctness_harris_shot_icp
	cdff_helpers cdff_logger cdff_types converters_opencv harris_detector_3d feature_extraction_3d shot_descriptor_3d feature_description_3d icp_3d feature_matching_3d mock pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters boost_system
)

add_executable(
    correctness_harris_shot_ransac
    Odometry3D/HarrisShotRansac.cpp
    Odometry3D/CorrectLocalizationTester.cpp
)
target_link_libraries(
	correctness_harris_shot_ransac
	cdff_helpers cdff_logger cdff_types converters_opencv harris_detector_3d feature_extraction_3d shot_descriptor_3d feature_description_3d ransac_3d feature_matching_3d mock pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters boost_system
)
