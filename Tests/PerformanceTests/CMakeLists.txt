add_executable(
    AsnTypesSizePrinter
    Common/AsnTypesSizePrinter.cpp
)
target_link_libraries(
	AsnTypesSizePrinter
	cdff_helpers cdff_logger cdff_types
)



add_library(
    performance_test_interface
    DFNs/PerformanceTestInterface.cpp
)
target_link_libraries(
	performance_test_interface
	cdff_helpers cdff_logger cdff_types
)


add_executable(
	hirschmuller_disparity_mapping_performance_test
	DFNs/StereoReconstruction/HirschmullerDisparityMapping.cpp
)
target_link_libraries(
	hirschmuller_disparity_mapping_performance_test
	mock cdff_helpers cdff_logger cdff_types hirschmuller_disparity_mapping performance_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)


add_library(
    performance_integration_test_interface
    DFNsIntegration/PerformanceTestInterface.cpp
)
target_link_libraries(
	performance_integration_test_interface
	cdff_helpers cdff_logger cdff_types
)


add_executable(
    orb_detector_descriptor_flann_matcher_performance_test
    DFNsIntegration/FeaturesExtractionDescriptionMatching/OrbDetectorDescriptorFlannMatcher.cpp
)
target_link_libraries(
	orb_detector_descriptor_flann_matcher_performance_test
	mock cdff_helpers cdff_logger cdff_types orb_detector_descriptor flann_matcher performance_integration_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)


add_executable(
    orb_flann_ransac_decomposition
    DFNsIntegration/VisualOdometry/OrbFlannRansacDecomposition.cpp
)
target_link_libraries(
	orb_flann_ransac_decomposition
	mock cdff_helpers cdff_logger cdff_types orb_detector_descriptor flann_matcher fundamental_matrix_ransac essential_matrix_decomposition performance_integration_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)


add_library(
    performance_dfpc_test_interface
    DFPCs/PerformanceTestInterface.cpp
)
target_link_libraries(
	performance_dfpc_test_interface
	cdff_helpers cdff_logger cdff_types
)


add_executable(
    reconstruction_from_stereo_performance_test
    DFPCs/Reconstruction3D/ReconstructionFromStereo.cpp
)
target_link_libraries(
	reconstruction_from_stereo_performance_test
	reconstruction_from_stereo mock converters_opencv cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui boost_system yaml-cpp reconstruction_3d_stub_observed_scene dfpc_configurator performance_dfpc_test_interface
)
