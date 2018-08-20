add_executable(
    registration_3d_icp_test
    Registration3D/Icp3D.cpp
    Registration3D/CorrectLocalizationTester.cpp
)
target_link_libraries(
	registration_3d_icp_test
	cdff_helpers cdff_logger cdff_types converters_opencv registration_3d registration_icp_3d opencv_core opencv_imgproc opencv_highgui boost_system pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters boost_system
)

add_executable(
    registration_3d_icp_cc_test
    Registration3D/IcpCC.cpp
    Registration3D/CorrectLocalizationTester.cpp
)
target_link_libraries(
	registration_3d_icp_cc_test
	cdff_helpers cdff_logger cdff_types converters_opencv registration_3d registration_icp_cc opencv_core opencv_imgproc opencv_highgui boost_system pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters boost_system
)

add_executable(
    position_test_svd
    BundleAdjustment/SvdDecomposition.cpp
    BundleAdjustment/PositionTester.cpp
)
target_link_libraries(
	position_test_svd
	cdff_helpers cdff_logger cdff_types converters_opencv svd_decomposition feature_extraction_2d opencv_core opencv_imgproc opencv_highgui boost_system
)

add_executable(
    validity_harris_detector_2d
    FeaturesExtraction2D/HarrisDetector2D.cpp
    FeaturesExtraction2D/SelectionTester.cpp
)
target_link_libraries(
	validity_harris_detector_2d
	cdff_helpers cdff_logger cdff_types converters_opencv harris_detector_2d feature_extraction_2d opencv_core opencv_imgproc opencv_highgui boost_system
)

add_executable(
    validity_orb_detector_2d
    FeaturesExtraction2D/OrbDetectorDescriptor.cpp
    FeaturesExtraction2D/SelectionTester.cpp
)
target_link_libraries(
	validity_orb_detector_2d
	cdff_helpers cdff_logger cdff_types converters_opencv orb_detector_descriptor feature_extraction_2d opencv_core opencv_imgproc opencv_highgui boost_system
)

add_executable(
    regularity_harris_detector_3d
    FeaturesExtraction3D/HarrisDetector3D.cpp
    FeaturesExtraction3D/RegularityTester.cpp
)
target_link_libraries(
	regularity_harris_detector_3d
	cdff_helpers cdff_logger cdff_types converters_opencv harris_detector_3d feature_extraction_3d pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters boost_system
)

add_executable(
    fundamental_matrix_ransac_test
    FundamentalMatrixComputation/FundamentalMatrixRansac.cpp
    FundamentalMatrixComputation/FundamentalMatrixTester.cpp
)
target_link_libraries(
	fundamental_matrix_ransac_test
	cdff_helpers cdff_logger cdff_types converters_opencv fundamental_matrix_computation fundamental_matrix_ransac opencv_core opencv_imgproc opencv_highgui boost_system
)

add_executable(
    linearity_image_undistortion
    ImageFiltering/ImageUndistortion.cpp
    ImageFiltering/LinearityTester.cpp
)
target_link_libraries(
	linearity_image_undistortion
	cdff_helpers cdff_logger cdff_types converters_opencv image_undistortion image_filtering opencv_core opencv_imgproc opencv_highgui boost_system
)

add_executable(
    quality_hirschmuller
    StereoReconstruction/HirschmullerDisparityMapping.cpp
    StereoReconstruction/QualityTester.cpp
)
target_link_libraries(
	quality_hirschmuller
	cdff_helpers cdff_logger cdff_types converters_opencv stereoReconstruction hirschmuller_disparity_mapping pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters opencv_core opencv_imgproc opencv_highgui boost_system
)

add_executable(
    quality_scanline
    StereoReconstruction/ScanlineOptimization.cpp
    StereoReconstruction/QualityTester.cpp
)
target_link_libraries(
	quality_scanline
	cdff_helpers cdff_logger cdff_types converters_opencv stereoReconstruction scanline_optimization pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters opencv_core opencv_imgproc opencv_highgui boost_system
)

add_executable(
    position_test_ceres
    BundleAdjustment/CeresAdjustment.cpp
    BundleAdjustment/PositionTester.cpp
)
target_link_libraries(
	position_test_ceres
	cdff_helpers cdff_logger cdff_types converters_opencv ceres_adjustment feature_extraction_2d opencv_core opencv_imgproc opencv_highgui boost_system ${CERES_LIBRARIES}
)