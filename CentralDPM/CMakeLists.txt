add_library(
    cdff_centraldpm
    CentralDPM.cpp
    RegisterMap.cpp
    RegisterPointcloud.cpp
)

target_include_directories(
  cdff_centraldpm
  SYSTEM PRIVATE ${ENVIRE_CORE_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_centraldpm
    PUBLIC cdff_types
    PRIVATE cdff_logger
    PRIVATE ${ENVIRE_CORE_LIBRARIES}
    PRIVATE class_loader 
)