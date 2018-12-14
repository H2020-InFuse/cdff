set(VISUAL_ODOMETRY_MAG_SOURCES "VisualOdometry_MAGInterface.cpp")
set(VISUAL_ODOMETRY_MAG_INCLUDE_DIRS "")
set(VISUAL_ODOMETRY_MAG_DEPENDENCIES "")

add_library(
    cdff_dfpc_visual_odometry_mag
    ${VISUAL_ODOMETRY_MAG_SOURCES}
    )
target_include_directories(
    cdff_dfpc_visual_odometry_mag
    SYSTEM PRIVATE ${VISUAL_ODOMETRY_MAG_INCLUDE_DIRS}
    )
target_link_libraries(
    cdff_dfpc_visual_odometry_mag
    ${VISUAL_ODOMETRY_MAG_DEPENDENCIES}
    )
