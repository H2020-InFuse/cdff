set(POSE_ESTIMATOR_SOURCES "PoseEstimatorInterface.cpp")
set(POSE_ESTIMATOR_INCLUDE_DIRS "")
set(POSE_ESTIMATOR_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
    set(POSE_ESTIMATOR_SOURCES ${POSE_ESTIMATOR_SOURCES} "WheeledRobotPoseEstimator.cpp" "PrimitivesPoseEstimator.cpp")
    #OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
    set(POSE_ESTIMATOR_DEPENDENCIES ${POSE_ESTIMATOR_DEPENDENCIES} ${OpenCV_LIBS} cdff_validators)
endif()

add_library(
        cdff_dfn_pose_estimator
        ${POSE_ESTIMATOR_SOURCES}
)
target_include_directories(
        cdff_dfn_pose_estimator
        SYSTEM PRIVATE ${POSE_ESTIMATOR_INCLUDE_DIRS}
)
target_link_libraries(
        cdff_dfn_pose_estimator
        ${POSE_ESTIMATOR_DEPENDENCIES}
)
