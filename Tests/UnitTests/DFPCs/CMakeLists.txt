add_executable(
    DFNsChainInterfaceTest
    DFNsChainInterface.cpp
)
target_link_libraries(
	DFNsChainInterfaceTest
	cdff_helpers cdff_logger dfns_chain_interface
)
add_test(DFNsChainInterfaceTest DFNsChainInterfaceTest)


add_executable(
    StructureFromMotionTest
    PointCloudModelLocalisation/StructureFromMotion.cpp
)
target_link_libraries(
	StructureFromMotionTest
	cdff_helpers cdff_logger structure_from_motion opencv_highgui opencv_core opencv_imgproc opencv_imgcodecs mock
)
add_test(StructureFromMotionTest StructureFromMotionTest)
