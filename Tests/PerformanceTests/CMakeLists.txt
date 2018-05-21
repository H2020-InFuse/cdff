
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

add_executable(
    harris_shot_icp
    DFNsIntegration/Odometry3D/HarrisShotIcp.cpp
    DFNsIntegration/Odometry3D/DetectionDescriptionMatching3D.cpp
)
target_link_libraries(
	harris_shot_icp
	mock cdff_helpers cdff_logger cdff_types harris_detector_3d shot_descriptor_3d icp_3d performance_integration_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)

add_executable(
    harris_shot_ransac
    DFNsIntegration/Odometry3D/HarrisShotRansac.cpp
    DFNsIntegration/Odometry3D/DetectionDescriptionMatching3D.cpp
)
target_link_libraries(
	harris_shot_ransac
	mock cdff_helpers cdff_logger cdff_types harris_detector_3d shot_descriptor_3d ransac_3d performance_integration_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
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
target_link_libraries(
	hirschmuller_disparity_mapping_performance_test
	mock cdff_helpers cdff_logger cdff_types hirschmuller_disparity_mapping performance_dfn_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)

add_executable(
	scanline_optimization_performance_test
	DFNs/StereoReconstruction/ScanlineOptimization.cpp
	DFNs/StereoReconstruction/StereoReconstruction.cpp
)
target_link_libraries(
	scanline_optimization_performance_test
	mock cdff_helpers cdff_logger cdff_types scanline_optimization performance_dfn_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)

add_executable(
	image_undistortion_rectification_performance_test
	DFNs/ImageFiltering/ImageUndistortionRectification.cpp
	DFNs/ImageFiltering/ImageFiltering.cpp
)
target_link_libraries(
	image_undistortion_rectification_performance_test
	mock cdff_helpers cdff_logger cdff_types image_undistortion_rectification performance_dfn_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)

add_executable(
	image_undistortion_performance_test
	DFNs/ImageFiltering/ImageUndistortion.cpp
	DFNs/ImageFiltering/ImageFiltering.cpp
)
target_link_libraries(
	image_undistortion_performance_test
	mock cdff_helpers cdff_logger cdff_types image_undistortion performance_dfn_test_interface cdff_opencv_visualizer cdff_pcl_visualizer
)


## DFPC performance tests
add_library(
    performance_dfpc_test_interface
    DFPCs/PerformanceTestInterface.cpp
)
target_link_libraries(
	performance_dfpc_test_interface
	cdff_helpers cdff_logger cdff_types performance_test_base
)

add_executable(
    reconstruction_from_stereo_performance_test
    DFPCs/Reconstruction3D/ReconstructionFromStereo.cpp
    DFPCs/Reconstruction3D/Reconstruction3D.cpp
)
target_link_libraries(
	reconstruction_from_stereo_performance_test
	reconstruction_from_stereo mock converters_opencv cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui boost_system yaml-cpp dfpc_configurator performance_dfpc_test_interface
)
