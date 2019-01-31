add_library(
    cdff_support_envireasn1
    RegisterPointcloud.cpp
    RegisterMap.cpp
)

target_include_directories(
    cdff_support_envireasn1
    SYSTEM PRIVATE ${ENVIRE_CORE_INCLUDE_DIRS}
)

target_link_libraries(
    cdff_support_envireasn1
    PUBLIC cdff_types
    PRIVATE ${ENVIRE_CORE_LIBRARIES} class_loader plugin_manager
)

# Copy the directory ${CMAKE_CURRENT_SOURCE_DIR}/plugin_manager
# into the directory ${CMAKE_CURRENT_BINARY_DIR}
file(COPY plugin_manager DESTINATION .)

# To be able to run anything that uses the Central Data Product Manager,
# the following directory:
#
#   ${CMAKE_CURRENT_SOURCE_DIR}/plugin_manager
#
# must be in a directory that is on the library search path. So unless the
# CDFF has been installed in a standard location, add the following path
# to your LD_LIBRARY_PATH environment variable:
#
#   ${CMAKE_CURRENT_BINARY_DIR}
#
# That is to say, at the time of writing:
#
#   ${CMAKE_BINARY_DIR}/Support/CentralDPM/EnvireASN1
