set(POINT_CLOUD_MODEL_LOCALISATION_SOURCES "PointCloudModelLocalisationInterface.cpp" "FeaturesMatching3D.cpp")
set(POINT_CLOUD_MODEL_LOCALISATION_INCLUDE_DIRS "")
set(POINT_CLOUD_MODEL_LOCALISATION_DEPENDENCIES "cdff_dfpc_configurator" "cdff_dfn_dfnexecutors")

add_library(
    cdff_dfpc_point_cloud_model_localisation
    ${POINT_CLOUD_MODEL_LOCALISATION_SOURCES}
    )
target_include_directories(
    cdff_dfpc_point_cloud_model_localisation 
    SYSTEM PRIVATE ${POINT_CLOUD_MODEL_LOCALISATION_INCLUDE_DIRS}
    )
target_link_libraries(
    cdff_dfpc_point_cloud_model_localisation
    ${POINT_CLOUD_MODEL_LOCALISATION_DEPENDENCIES}
    )
