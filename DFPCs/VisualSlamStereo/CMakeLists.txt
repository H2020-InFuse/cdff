set(VISUAL_SLAM_STEREO_SOURCES "VisualSlamStereo.cpp")
set(VISUAL_SLAM_STEREO_INCLUDE_DIRS "")
set(VISUAL_SLAM_STEREO_DEPENDENCIES "")

add_library(
    cdff_dfpc_visual_slam_stereo
    ${VISUAL_SLAM_STEREO_SOURCES}
    )
target_include_directories(
    cdff_dfpc_visual_slam_stereo
    SYSTEM PRIVATE ${VISUAL_SLAM_STEREO_INCLUDE_DIRS}
    )
target_link_libraries(
    cdff_dfpc_visual_slam_stereo
    ${VISUAL_SLAM_STEREO_DEPENDENCIES}
    )
