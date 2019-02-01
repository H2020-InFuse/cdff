add_subdirectory(EnvireASN1)

add_library(
    cdff_support_centraldpm
    CentralDPM.cpp
)

target_include_directories(
    cdff_support_centraldpm
    SYSTEM PRIVATE ${ENVIRE_CORE_INCLUDE_DIRS}
)

target_link_libraries(
    cdff_support_centraldpm
    PUBLIC cdff_types
    PRIVATE cdff_logger ${ENVIRE_CORE_LIBRARIES} class_loader cdff_support_envireasn1
)
