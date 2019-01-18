add_library(
    envire_asn1
    RegisterPointcloud.cpp
    RegisterMap.cpp
)

target_include_directories(
    envire_asn1
    SYSTEM PRIVATE ${ENVIRE_CORE_INCLUDE_DIRS} PluginManager
)

target_link_libraries(
    envire_asn1
    PUBLIC cdff_types
    PRIVATE ${ENVIRE_CORE_LIBRARIES} class_loader plugin_manager
    #SYSTEM PRIVATE ${ENVIRE_CORE_LIBRARIES} class_loader plugin_manager
)