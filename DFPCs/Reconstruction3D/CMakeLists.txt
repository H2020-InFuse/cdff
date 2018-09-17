add_library(
    reconstruction_3d
    Reconstruction3DInterface.cpp
)
target_link_libraries(
	reconstruction_3d
)
set_target_properties(reconstruction_3d PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    dense_registration_from_stereo
    DenseRegistrationFromStereo.cpp
    BundleHistory.cpp
    PointCloudMap.cpp
)
target_link_libraries(
    dense_registration_from_stereo
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
)
set_target_properties(dense_registration_from_stereo PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    sparse_registration_from_stereo
    SparseRegistrationFromStereo.cpp
    BundleHistory.cpp
    PointCloudMap.cpp
)
target_link_libraries(
    sparse_registration_from_stereo
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
)
set_target_properties(sparse_registration_from_stereo PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    adjustment_from_stereo
    AdjustmentFromStereo.cpp
    PointCloudMap.cpp
    BundleHistory.cpp
    MultipleCorrespondences2DRecorder.cpp
)
target_link_libraries(
    adjustment_from_stereo
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
)
set_target_properties(adjustment_from_stereo PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    reconstruction_from_motion
    ReconstructionFromMotion.cpp
    ObservedScene.cpp
)
target_link_libraries(
    reconstruction_from_motion
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
)
set_target_properties(reconstruction_from_motion PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    reconstruction_from_stereo
    ReconstructionFromStereo.cpp
    PointCloudMap.cpp
    BundleHistory.cpp
)
target_link_libraries(
    reconstruction_from_stereo
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
)
set_target_properties(reconstruction_from_stereo PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    registration_from_stereo
    RegistrationFromStereo.cpp
    PointCloudMap.cpp
    BundleHistory.cpp
)
target_link_libraries(
    registration_from_stereo
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
)
set_target_properties(registration_from_stereo PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    estimation_from_stereo
    EstimationFromStereo.cpp
    PointCloudMap.cpp
    BundleHistory.cpp
    MultipleCorrespondencesRecorder.cpp
)
target_link_libraries(
    estimation_from_stereo
    reconstruction_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
)
set_target_properties(estimation_from_stereo PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

