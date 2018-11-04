
add_library(
    point_cloud_model_localisation
    PointCloudModelLocalisationInterface.cpp
)
target_link_libraries(
	point_cloud_model_localisation
)

add_library(
    dfpc_implementation_features_matching_3d
    FeaturesMatching3D.cpp
)
target_link_libraries(
    dfpc_implementation_features_matching_3d
    point_cloud_model_localisation cdff_types yaml-cpp cdff_helpers cdff_visualizers_opencv cdff_visualizers_pcl dfpc_configurator
)
