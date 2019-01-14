set(MODEL_TRACKER_SOURCES "MidRange3DModelTrackingInterface.cpp" "WheelTracker.cpp" "WheeledRobotTracker.cpp")
set(MODEL_TRACKER_INCLUDE_DIRS "")
set(MODEL_TRACKER_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters" cdff_dfn_image_filtering cdff_dfn_primitive_finder cdff_dfn_pose_estimator cdff_dfn_weighting_expert)

add_library(
        cdff_dfpc_model_based_tracker
        ${MODEL_TRACKER_SOURCES}
)
target_include_directories(
        cdff_dfpc_model_based_tracker
        SYSTEM PRIVATE ${MODEL_TRACKER_INCLUDE_DIRS}
)
target_link_libraries(
        cdff_dfpc_model_based_tracker
        ${MODEL_TRACKER_DEPENDENCIES}
)