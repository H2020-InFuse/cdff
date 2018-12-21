set(POSE_FUSION_SOURCES "PoseFusionInterface.cpp")
set(POSE_FUSION_INCLUDE_DIRS "")
set(POSE_FUSION_DEPENDENCIES "")

add_library(
    cdff_dfpc_pose_fusion
    ${POSE_FUSION_SOURCES}
    )
target_include_directories(
    cdff_dfpc_pose_fusion
    SYSTEM PRIVATE ${POSE_FUSION_INCLUDE_DIRS}
    )
target_link_libraries(
    cdff_dfpc_pose_fusion
    ${POSE_FUSION_DEPENDENCIES}
    )
