add_executable(
    harris_icp_cc
    FeaturesRegistration/HarrisIcpCC.cpp
    FeaturesRegistration/CorrectLocalizationTester.cpp
)
target_compile_definitions(harris_icp_cc PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	harris_icp_cc
	cdff_helpers cdff_logger cdff_types cdff_converters cdff_dfn_features_extraction_3d cdff_dfn_registration_3d opencv_core opencv_imgproc opencv_highgui pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters
)

add_executable(
    ransac_essential_triangulation_test
    FeaturesReconstruction/EssentialMatrixDecompositionTriangulation.cpp
    FeaturesReconstruction/ReconstructionTester.cpp
)
target_link_libraries(
	ransac_essential_triangulation_test
	cdff_helpers cdff_logger cdff_types cdff_converters cdff_dfn_fundamental_matrix_computation cdff_dfn_cameras_transform_estimation cdff_dfn_point_cloud_reconstruction_2d_to_3d opencv_core opencv_imgproc opencv_highgui pcl_common
)

add_executable(
    validity_orb_flann
    Matching2D/OrbFlann.cpp
    Matching2D/SelectionTester.cpp
)
target_link_libraries(
	validity_orb_flann
	cdff_helpers cdff_logger cdff_types cdff_converters cdff_dfn_features_description_2d cdff_dfn_features_matching_2d opencv_core opencv_imgproc opencv_highgui
)

add_executable(
    validity_shot_icp
    Matching3D/ShotIcp.cpp
    Matching3D/SelectionTester.cpp
)
target_compile_definitions(validity_shot_icp PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	validity_shot_icp
	cdff_helpers cdff_logger cdff_types cdff_converters cdff_dfn_features_description_3d cdff_dfn_features_matching_3d  pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters
)

add_executable(
    validity_shot_ransac
    Matching3D/ShotRansac.cpp
    Matching3D/SelectionTester.cpp
)
target_compile_definitions(validity_shot_ransac PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	validity_shot_ransac
	cdff_helpers cdff_logger cdff_types cdff_converters cdff_dfn_features_description_3d cdff_dfn_features_matching_3d pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters
)

add_executable(
    correctness_harris_shot_icp
    Odometry3D/HarrisShotIcp.cpp
    Odometry3D/CorrectLocalizationTester.cpp
)
target_compile_definitions(correctness_harris_shot_icp PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	correctness_harris_shot_icp
	cdff_helpers cdff_logger cdff_types cdff_converters cdff_dfn_features_extraction_3d cdff_dfn_features_description_3d cdff_dfn_features_matching_3d pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters cdff_visualizers_pcl
)

add_executable(
    correctness_harris_shot_ransac
    Odometry3D/HarrisShotRansac.cpp
    Odometry3D/CorrectLocalizationTester.cpp
)
target_compile_definitions(correctness_harris_shot_ransac PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	correctness_harris_shot_ransac
	cdff_helpers cdff_logger cdff_types cdff_converters cdff_dfn_features_extraction_3d cdff_dfn_features_description_3d cdff_dfn_features_matching_3d pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters cdff_visualizers_pcl
)

add_executable(
    correctness_nearest_neighbour_average
    PointCloudAssembly/NearestNeighbourAverage.cpp
    PointCloudAssembly/CorrectAssemblyTester.cpp
)
target_compile_definitions(correctness_nearest_neighbour_average PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	correctness_nearest_neighbour_average
	cdff_helpers cdff_logger cdff_types cdff_converters cdff_dfn_point_cloud_assembly cdff_dfn_point_cloud_transform pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters pcl_kdtree cdff_dfn_dfnexecutors
)

add_executable(
    correctness_voxel_binning
    PointCloudAssembly/VoxelBinning.cpp
    PointCloudAssembly/CorrectAssemblyTester.cpp
)
target_compile_definitions(correctness_voxel_binning PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	correctness_voxel_binning
	cdff_helpers cdff_logger cdff_types cdff_converters cdff_dfn_point_cloud_assembly cdff_dfn_point_cloud_transform pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters pcl_kdtree cdff_dfn_dfnexecutors
)


