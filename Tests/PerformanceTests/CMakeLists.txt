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
