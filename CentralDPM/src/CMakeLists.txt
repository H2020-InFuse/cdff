add_library(
    centralDPM
    CentralDPM.cpp
)

target_link_libraries(
    centralDPM
    cdff_types
    ${ENVIRE_CORE_LIBRARIES}
    class_loader 
)