add_library(
    reconstruction_3d
    Reconstruction3DInterface.cpp
)

target_link_libraries(
	reconstruction_3d
	dfns_chain_interface
)

add_library(
    reconstruction_from_motion
    ReconstructionFromMotion.cpp
)
target_link_libraries(
    reconstruction_from_motion
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer
)

add_library(
    reconstruction_from_stereo
    ReconstructionFromStereo.cpp
)
target_link_libraries(
    reconstruction_from_stereo
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer
)
