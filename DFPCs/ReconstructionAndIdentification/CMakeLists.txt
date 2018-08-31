
add_library(
    reconstruction_and_identification
    ReconstructionAndIdentificationInterface.cpp
)
target_link_libraries(
	reconstruction_and_identification
)

add_library(
    registration_and_matching
    RegistrationAndMatching.cpp
)
target_link_libraries(
    registration_and_matching
    reconstruction_and_identification registration_from_stereo dfpc_implementation_features_matching_3d cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
)
