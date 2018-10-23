
set(CMAKE_CXX_FLAGS "-g")

add_library(
    central_dpm
    CentralDPM.cpp
)

target_link_libraries(
    central_dpm
    cdff_types
    cdff_logger
    ${ENVIRE_CORE_LIBRARIES}
    class_loader 
)