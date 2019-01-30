set(POSE_WEIGHTING_SOURCES "PoseWeightingInterface.cpp")
set(POSE_WEIGHTING_INCLUDE_DIRS "")
set(POSE_WEIGHTING_DEPENDENCIES "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
    set(POSE_WEIGHTING_SOURCES ${POSE_WEIGHTING_SOURCES} "KalmanFilter.cpp")
    #OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
    set(POSE_WEIGHTING_DEPENDENCIES ${POSE_WEIGHTING_DEPENDENCIES} opencv_highgui opencv_video)
endif()

add_library(
        cdff_dfn_pose_weighting
        ${POSE_WEIGHTING_SOURCES}
)
target_include_directories(
        cdff_dfn_pose_weighting
        SYSTEM PRIVATE ${POSE_WEIGHTING_INCLUDE_DIRS}
)
target_link_libraries(
        cdff_dfn_pose_weighting
        ${POSE_WEIGHTING_DEPENDENCIES}
)