add_library(
    absolute_localization
    AbsoluteLocalizationInterface.cpp
)

target_link_libraries(
    absolute_localization
    cdff_types yaml-cpp cdff_helpers dfpc_configurator
)
