add_library(
    point_cloud_model_localisation
    PointCloudModelLocalisationInterface.cpp
)

target_link_libraries(
	point_cloud_model_localisation
	dfns_chain_interface
)

add_library(
    structure_from_motion
    StructureFromMotion.cpp
)
target_link_libraries(
    structure_from_motion
    point_cloud_model_localisation cdff_types yaml-cpp cdff_helpers
)
