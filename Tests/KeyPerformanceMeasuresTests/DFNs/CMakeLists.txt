add_executable(
    registration_3d_icp_test
    Registration3D/Icp3D.cpp
    Registration3D/CorrectLocalizationTester.cpp
)
target_compile_definitions(registration_3d_icp_test PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	registration_3d_icp_test
	cdff_helpers cdff_logger cdff_types cdff_converters cdff_dfn_registration_3d opencv_core opencv_imgproc opencv_highgui pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters
)

add_executable(
    registration_3d_icp_cc_test
    Registration3D/IcpCC.cpp
    Registration3D/CorrectLocalizationTester.cpp
)
target_compile_definitions(registration_3d_icp_cc_test PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	registration_3d_icp_cc_test
	cdff_helpers cdff_logger cdff_types cdff_converters cdff_dfn_registration_3d opencv_core opencv_imgproc opencv_highgui pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters
)

add_executable(
    registration_3d_icp_matcher_test
    Registration3D/IcpMatcher.cpp
    Registration3D/CorrectLocalizationTester.cpp
)
target_link_libraries(
	registration_3d_icp_matcher_test
	cdff_helpers cdff_logger cdff_types cdff_converters cdff_dfn_registration_3d opencv_core opencv_imgproc opencv_highgui boost_system pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters boost_system
)

add_executable(
    position_test_svd
    BundleAdjustment/SvdDecomposition.cpp
    BundleAdjustment/PositionTester.cpp
)
target_link_libraries(
	position_test_svd
	cdff_helpers cdff_logger cdff_types cdff_converters svd_decomposition feature_extraction_2d opencv_core opencv_imgproc opencv_highgui
)

add_executable(
    validity_harris_detector_2d
    FeaturesExtraction2D/HarrisDetector2D.cpp
    FeaturesExtraction2D/SelectionTester.cpp
)
target_link_libraries(
	validity_harris_detector_2d
	cdff_helpers cdff_logger cdff_types cdff_converters harris_detector_2d feature_extraction_2d opencv_core opencv_imgproc opencv_highgui
)

add_executable(
    validity_orb_detector_2d
    FeaturesExtraction2D/OrbDetectorDescriptor.cpp
    FeaturesExtraction2D/SelectionTester.cpp
)
target_link_libraries(
	validity_orb_detector_2d
	cdff_helpers cdff_logger cdff_types cdff_converters orb_detector_descriptor feature_extraction_2d opencv_core opencv_imgproc opencv_highgui
)

add_executable(
    regularity_harris_detector_3d
    FeaturesExtraction3D/HarrisDetector3D.cpp
    FeaturesExtraction3D/RegularityTester.cpp
)
target_compile_definitions(regularity_harris_detector_3d PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	regularity_harris_detector_3d
	cdff_helpers cdff_logger cdff_types cdff_converters harris_detector_3d feature_extraction_3d pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters
)

add_executable(
    fundamental_matrix_ransac_test
    FundamentalMatrixComputation/FundamentalMatrixRansac.cpp
    FundamentalMatrixComputation/FundamentalMatrixTester.cpp
)
target_link_libraries(
	fundamental_matrix_ransac_test
	cdff_helpers cdff_logger cdff_types cdff_converters fundamental_matrix_computation fundamental_matrix_ransac opencv_core opencv_imgproc opencv_highgui
)

add_executable(
    linearity_image_undistortion
    ImageFiltering/ImageUndistortion.cpp
    ImageFiltering/LinearityTester.cpp
)
target_link_libraries(
	linearity_image_undistortion
	cdff_helpers cdff_logger cdff_types cdff_converters cdff_dfn_image_filtering opencv_core opencv_imgproc opencv_highgui
)

add_executable(
    quality_hirschmuller
    StereoReconstruction/HirschmullerDisparityMapping.cpp
    StereoReconstruction/QualityTester.cpp
)
target_compile_definitions(quality_hirschmuller PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	quality_hirschmuller
	cdff_helpers cdff_logger cdff_types cdff_converters cdff_dfn_stereo_reconstruction pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters opencv_core opencv_imgproc opencv_highgui cdff_visualizers_opencv cdff_visualizers_pcl
)

add_executable(
    quality_scanline
    StereoReconstruction/ScanlineOptimization.cpp
    StereoReconstruction/QualityTester.cpp
)
target_compile_definitions(quality_scanline PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	quality_scanline
	cdff_helpers cdff_logger cdff_types cdff_converters cdff_dfn_stereo_reconstruction pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters opencv_core opencv_imgproc opencv_highgui cdff_visualizers_opencv cdff_visualizers_pcl
)

add_executable(
    position_test_ceres
    BundleAdjustment/CeresAdjustment.cpp
    BundleAdjustment/PositionTester.cpp
)
target_link_libraries(
	position_test_ceres
	cdff_helpers cdff_logger cdff_types cdff_converters ceres_adjustment feature_extraction_2d opencv_core opencv_imgproc opencv_highgui ${CERES_LIBRARIES}
)
