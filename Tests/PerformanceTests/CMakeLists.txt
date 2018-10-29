
## AsnTypesSize Printer
add_executable(
    AsnTypesSizePrinter
    Common/AsnTypesSizePrinter.cpp
)
target_link_libraries(
	AsnTypesSizePrinter
	cdff_helpers cdff_logger cdff_types
)


##Performance Test Base
add_library(
    performance_test_base
    PerformanceTestBase.cpp
    Aggregator.cpp
)
target_link_libraries(
	performance_test_base
	cdff_helpers cdff_logger cdff_types
)



## Integration Performance Tests
add_library(
    performance_integration_test_interface
    DFNsIntegration/PerformanceTestInterface.cpp
)
target_link_libraries(
	performance_integration_test_interface
	cdff_helpers cdff_logger cdff_types performance_test_base
)

add_executable(
    orb_detector_descriptor_flann_matcher_performance_test
    DFNsIntegration/FeaturesExtractionDescriptionMatching/OrbDetectorDescriptorFlannMatcher.cpp
)
target_link_libraries(
	orb_detector_descriptor_flann_matcher_performance_test
	cdff_helpers cdff_logger cdff_types orb_detector_descriptor flann_matcher performance_integration_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)

add_executable(
    orb_flann_ransac_decomposition
    DFNsIntegration/VisualOdometry/OrbFlannRansacDecomposition.cpp
)
target_link_libraries(
	orb_flann_ransac_decomposition
	cdff_helpers cdff_logger cdff_types orb_detector_descriptor flann_matcher fundamental_matrix_ransac essential_matrix_decomposition performance_integration_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)

add_executable(
    harris_shot_icp
    DFNsIntegration/Odometry3D/HarrisShotIcp.cpp
    DFNsIntegration/Odometry3D/DetectionDescriptionMatching3D.cpp
)
target_compile_definitions(harris_shot_icp PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	harris_shot_icp
	cdff_helpers cdff_logger cdff_types harris_detector_3d shot_descriptor_3d icp_3d performance_integration_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)

add_executable(
    harris_shot_ransac
    DFNsIntegration/Odometry3D/HarrisShotRansac.cpp
    DFNsIntegration/Odometry3D/DetectionDescriptionMatching3D.cpp
)
target_compile_definitions(harris_shot_ransac PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	harris_shot_ransac
	cdff_helpers cdff_logger cdff_types harris_detector_3d shot_descriptor_3d ransac_3d performance_integration_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)


## DFN performance Tests
add_library(
    performance_dfn_test_interface
    DFNs/PerformanceTestInterface.cpp
)
target_link_libraries(
	performance_dfn_test_interface
	cdff_helpers cdff_logger cdff_types performance_test_base
)

add_executable(
	hirschmuller_disparity_mapping_performance_test
	DFNs/StereoReconstruction/HirschmullerDisparityMapping.cpp
	DFNs/StereoReconstruction/StereoReconstruction.cpp
)
target_compile_definitions(hirschmuller_disparity_mapping_performance_test PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	hirschmuller_disparity_mapping_performance_test
	cdff_helpers cdff_logger cdff_types hirschmuller_disparity_mapping performance_dfn_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)

add_executable(
	scanline_optimization_performance_test
	DFNs/StereoReconstruction/ScanlineOptimization.cpp
	DFNs/StereoReconstruction/StereoReconstruction.cpp
)
target_compile_definitions(scanline_optimization_performance_test PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	scanline_optimization_performance_test
	cdff_helpers cdff_logger cdff_types scanline_optimization performance_dfn_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)

add_executable(
	image_undistortion_rectification_performance_test
	DFNs/ImageFiltering/ImageUndistortionRectification.cpp
	DFNs/ImageFiltering/ImageFiltering.cpp
)
target_link_libraries(
	image_undistortion_rectification_performance_test
	cdff_helpers cdff_logger cdff_types image_undistortion_rectification performance_dfn_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)

add_executable(
	image_undistortion_performance_test
	DFNs/ImageFiltering/ImageUndistortion.cpp
	DFNs/ImageFiltering/ImageFiltering.cpp
)
target_link_libraries(
	image_undistortion_performance_test
	cdff_helpers cdff_logger cdff_types image_undistortion performance_dfn_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)
