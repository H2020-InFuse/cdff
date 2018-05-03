
## AsnTypesSize Printer
add_executable(
    regularity_harris_detector_3d
    DFNs/FeaturesExtraction3D/HarrisDetector3D.cpp
    DFNs/FeaturesExtraction3D/RegularityTester.cpp
)
target_link_libraries(
	regularity_harris_detector_3d
	cdff_helpers cdff_logger cdff_types converters_opencv harris_detector_3d feature_extraction_3d mock pcl_common pcl_io pcl_io_ply pcl_search pcl_keypoints pcl_features pcl_filters boost_system
)
