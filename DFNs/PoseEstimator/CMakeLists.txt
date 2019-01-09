add_library(
    pose_estimator
    PoseEstimatorInterface.cpp
)

add_library(
    primitives_pose_estimator
    PrimitivesPoseEstimator.cpp
)
target_link_libraries(
    primitives_pose_estimator
    pose_estimator cdff_types yaml-cpp cdff_helpers ${OpenCV_LIBS}
)

add_library(
    wheeled_robot_pose_estimator
    WheeledRobotPoseEstimator.cpp
)
target_link_libraries(
    wheeled_robot_pose_estimator
    pose_estimator cdff_types yaml-cpp cdff_helpers ${OpenCV_LIBS}
)
