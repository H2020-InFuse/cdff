set(LIDAR_MAP_BASED_LOCALIZATION_SOURCES "LIDARMapBasedLocalizationInterface.cpp")
set(LIDAR_MAP_BASED_LOCALIZATION_INCLUDE_DIRS "")
set(LIDAR_MAP_BASED_LOCALIZATION_DEPENDENCIES "")

add_library(
    cdff_dfpc_lidar_map_based_localization
    ${LIDAR_MAP_BASED_LOCALIZATION_SOURCES}
    )
target_include_directories(
    cdff_dfpc_lidar_map_based_localization
    SYSTEM PRIVATE ${LIDAR_MAP_BASED_LOCALIZATION_INCLUDE_DIRS}
    )
target_link_libraries(
    cdff_dfpc_lidar_map_based_localization
    ${LIDAR_MAP_BASED_LOCALIZATION_DEPENDENCIES}
    )
