add_library(
    envire_asn1
    RegisterPointcloud.cpp
    RegisterMap.cpp
)

target_include_directories(
    envire_asn1
    SYSTEM PRIVATE ${ENVIRE_CORE_INCLUDE_DIRS} 
)

target_link_libraries(
    envire_asn1
    PUBLIC cdff_types
    PRIVATE ${ENVIRE_CORE_LIBRARIES} class_loader plugin_manager
)

# The DESTINATION folder below must be appended to the environmental variable $LD_LIBRARY_PATH
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/plugin_manager 
    DESTINATION ${CMAKE_BINARY_DIR}/CentralDPM/Envire_asn1)