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

add_executable(
    quality_hirschmuller
    DFNs/StereoReconstruction/HirschmullerDisparityMapping.cpp
    DFNs/StereoReconstruction/QualityTester.cpp
)
target_link_libraries(
	quality_hirschmuller
	cdff_helpers cdff_logger cdff_types converters_opencv stereoReconstruction hirschmuller_disparity_mapping mock pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters opencv_core opencv_imgproc opencv_highgui boost_system
)

add_executable(
    fundamental_matrix_ransac_test
    DFNs/FundamentalMatrixComputation/FundamentalMatrixRansac.cpp
    DFNs/FundamentalMatrixComputation/FundamentalMatrixTester.cpp
)
target_link_libraries(
	fundamental_matrix_ransac_test
	cdff_helpers cdff_logger cdff_types converters_opencv fundamental_matrix_computation fundamental_matrix_ransac mock opencv_core opencv_imgproc opencv_highgui boost_system
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

add_executable(
    validity_shot_icp
    DFNsIntegration/Matching3D/ShotIcp.cpp
    DFNsIntegration/Matching3D/SelectionTester.cpp
)
target_link_libraries(
	validity_shot_icp
	cdff_helpers cdff_logger cdff_types converters_opencv shot_descriptor_3d feature_description_3d icp_3d feature_matching_3d mock pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters boost_system
)

add_executable(
    ransac_essential_triangulation_test
    DFNsIntegration/FeaturesReconstruction/EssentialMatrixDecompositionTriangulation.cpp
    DFNsIntegration/FeaturesReconstruction/ReconstructionTester.cpp
)
target_link_libraries(
	ransac_essential_triangulation_test
	cdff_helpers cdff_logger cdff_types converters_opencv fundamental_matrix_ransac essential_matrix_decomposition triangulation mock opencv_core opencv_imgproc opencv_highgui boost_system pcl_common
)


#DFPC
add_executable(
    quality_registration_from_stereo
    DFPCs/Reconstruction3D/RegistrationFromStereo.cpp
    DFPCs/Reconstruction3D/ReconstructionExecutor.cpp
)
target_link_libraries(
	quality_registration_from_stereo
	registration_from_stereo mock converters_opencv cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui boost_system yaml-cpp dfpc_configurator
)
