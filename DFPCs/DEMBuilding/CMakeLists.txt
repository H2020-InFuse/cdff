set(DEM_BUILDING_SOURCES "DEMBuildingInterface.cpp")
set(DEM_BUILDING_INCLUDE_DIRS "")
set(DEM_BUILDING_DEPENDENCIES "")

add_library(
    cdff_dfpc_dem_building
    ${DEM_BUILDING_SOURCES}
    )
target_include_directories(
    cdff_dfpc_dem_building
    SYSTEM PRIVATE ${DEM_BUILDING_INCLUDE_DIRS}
    )
target_link_libraries(
    cdff_dfpc_dem_building
    ${DEM_BUILDING_DEPENDENCIES}
    )
