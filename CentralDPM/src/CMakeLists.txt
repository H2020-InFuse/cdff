add_library(
    cdff_centraldpm
    CentralDPM.cpp
)

target_include_directories(
  cdff_centraldpm
  PRIVATE SYSTEM ${ENVIRE_CORE_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_centraldpm
    cdff_types
    cdff_logger
    ${ENVIRE_CORE_LIBRARIES}
    class_loader 
)