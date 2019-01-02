set(REGISTRATION_AND_IDENTIFICATION_SOURCES "ReconstructionAndIdentificationInterface.cpp" "RegistrationAndMatching.cpp")
set(REGISTRATION_AND_IDENTIFICATION_INCLUDE_DIRS "")
set(REGISTRATION_AND_IDENTIFICATION_DEPENDENCIES "cdff_dfpc_configurator" "cdff_dfpc_reconstruction_3d" "cdff_dfpc_point_cloud_model_localisation")

add_library(
    cdff_dfpc_reconstruction_and_identification
    ${REGISTRATION_AND_IDENTIFICATION_SOURCES}
    )
target_include_directories(
    cdff_dfpc_reconstruction_and_identification
    SYSTEM PRIVATE ${REGISTRATION_AND_IDENTIFICATION_INCLUDE_DIRS}
    )
target_link_libraries(
    cdff_dfpc_reconstruction_and_identification
    ${REGISTRATION_AND_IDENTIFICATION_DEPENDENCIES}
    )
