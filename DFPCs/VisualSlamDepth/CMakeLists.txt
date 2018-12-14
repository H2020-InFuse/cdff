set(VISUAL_SLAM_SOURCES "VisualSlamDepth.cpp")
set(VISUAL_SLAM_INCLUDE_DIRS "")
set(VISUAL_SLAM_DEPENDENCIES "")

add_library(
    cdff_dfpc_visual_slam_depth
    ${VISUAL_SLAM_SOURCES}
    )
target_include_directories(
    cdff_dfpc_visual_slam_depth
    SYSTEM PRIVATE ${VISUAL_SLAM_INCLUDE_DIRS}
    )
target_link_libraries(
    cdff_dfpc_visual_slam_depth
    ${VISUAL_SLAM_DEPENDENCIES}
    )
