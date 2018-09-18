
add_library(
    visual_odometry_LAAS
    VisualOdometry_LAASInterface.cpp
)

target_link_libraries(
    visual_odometry_LAAS
    #cdff_types yaml-cpp
)
