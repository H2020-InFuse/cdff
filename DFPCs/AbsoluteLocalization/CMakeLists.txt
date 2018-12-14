set(ABSOLUTE_LOCALIZATION_SOURCES "AbsoluteLocalizationInterface.cpp")
set(ABSOLUTE_LOCALIZATION_INCLUDE_DIRS "")
set(ABSOLUTE_LOCALIZATION_DEPENDENCIES "cdff_dfpc_configurator" "cdff_helpers" "cdff_types" "yaml-cpp")

add_library(
    cdff_dfpc_absolute_localization
    ${ABSOLUTE_LOCALIZATION_SOURCES}
    )
target_include_directories(
    cdff_dfpc_absolute_localization
    SYSTEM PRIVATE ${ABSOLUTE_LOCALIZATION_INCLUDE_DIRS}
    )
target_link_libraries(
    cdff_dfpc_absolute_localization
    ${ABSOLUTE_LOCALIZATION_DEPENDENCIES}
    )
