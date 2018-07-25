add_library(
    reconstruction_3d
    Reconstruction3DInterface.cpp
)
target_link_libraries(
	reconstruction_3d
)

add_library(
    dense_registration_from_stereo
    DenseRegistrationFromStereo.cpp
    PointCloudMap.cpp
)
target_link_libraries(
    dense_registration_from_stereo
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
)

add_library(
    sparse_registration_from_stereo
    SparseRegistrationFromStereo.cpp
    PointCloudMap.cpp
)
target_link_libraries(
    sparse_registration_from_stereo
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
)

add_library(
    adjustment_from_stereo
    AdjustmentFromStereo.cpp
    PointCloudMap.cpp
)
target_link_libraries(
    adjustment_from_stereo
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
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

add_library(
    estimation_from_stereo
    EstimationFromStereo.cpp
    PointCloudMap.cpp
)
target_link_libraries(
    estimation_from_stereo
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
)


