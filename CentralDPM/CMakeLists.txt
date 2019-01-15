add_library(
    envire_asn1
    RegisterPointcloud.cpp
    RegisterMap.cpp
)

target_include_directories(
    envire_asn1
    SYSTEM PRIVATE ${ENVIRE_CORE_INCLUDE_DIRS} PluginManager
)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/envire_asn1.xml ${_DEPENDENCIES_PREFIX}/lib/plugin_manager/envire_asn1.xml COPYONLY)

target_link_libraries(
    envire_asn1
    PUBLIC cdff_types
    PRIVATE ${ENVIRE_CORE_LIBRARIES}
    PRIVATE class_loader 
    PRIVATE plugin_manager
)

add_library(
    cdff_centraldpm
    CentralDPM.cpp
)

target_include_directories(
  cdff_centraldpm
  SYSTEM PRIVATE ${ENVIRE_CORE_INCLUDE_DIRS}
  SYSTEM PRIVATE envire_asn1
)

target_link_libraries(
    cdff_centraldpm
    PUBLIC cdff_types
    PRIVATE cdff_logger
    PRIVATE ${ENVIRE_CORE_LIBRARIES}
    PRIVATE class_loader 
    PRIVATE envire_asn1
)