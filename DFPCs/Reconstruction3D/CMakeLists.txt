add_library(
    reconstruction_3d
    Reconstruction3DInterface.cpp
)

target_link_libraries(
	reconstruction_3d
)

add_library(
    reconstruction_from_motion
    ReconstructionFromMotion.cpp
    ObservedScene.cpp
)
target_link_libraries(
    reconstruction_from_motion
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
)

add_library(
    reconstruction_from_stereo
    ReconstructionFromStereo.cpp
    ObservedScene.cpp
)
target_link_libraries(
    reconstruction_from_stereo
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
)

add_library(
    registration_from_stereo
    RegistrationFromStereo.cpp
    PointCloudMap.cpp
)
target_link_libraries(
    registration_from_stereo
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
)
