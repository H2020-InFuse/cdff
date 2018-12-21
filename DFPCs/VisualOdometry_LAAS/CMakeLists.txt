set(VISUAL_ODOMETRY_LAAS_SOURCES "VisualOdometry_LAASInterface.cpp")
set(VISUAL_ODOMETRY_LAAS_INCLUDE_DIRS "")
set(VISUAL_ODOMETRY_LAAS_DEPENDENCIES "")

add_library(
    cdff_dfpc_visual_odometry_laas
    ${VISUAL_ODOMETRY_LAAS_SOURCES}
    )
target_include_directories(
    cdff_dfpc_visual_odometry_laas
    SYSTEM PRIVATE ${VISUAL_ODOMETRY_LAAS_INCLUDE_DIRS}
    )
target_link_libraries(
    cdff_dfpc_visual_odometry_laas
    ${VISUAL_ODOMETRY_LAAS_DEPENDENCIES}
    )
