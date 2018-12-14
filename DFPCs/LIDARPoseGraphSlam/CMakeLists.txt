set(LIDAR_POSE_GRAPH_SLAM_SOURCES "LIDARPoseGraphSlam.cpp")
set(LIDAR_POSE_GRAPH_SLAM_INCLUDE_DIRS "")
set(LIDAR_POSE_GRAPH_SLAM_DEPENDENCIES "")

add_library(
    cdff_dfpc_lidar_pose_graph_slam
    ${LIDAR_POSE_GRAPH_SLAM_SOURCES}
    )
target_include_directories(
    cdff_dfpc_lidar_pose_graph_slam
    SYSTEM PRIVATE ${LIDAR_POSE_GRAPH_SLAM_INCLUDE_DIRS}
    )
target_link_libraries(
    cdff_dfpc_lidar_pose_graph_slam
    ${LIDAR_POSE_GRAPH_SLAM_DEPENDENCIES}
    )
