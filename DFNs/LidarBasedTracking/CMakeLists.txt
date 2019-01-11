set(LIDAR_BASED_TRACKING_SOURCES "LidarBasedTrackingInterface.cpp")
set(LIDAR_BASED_TRACKING_INCLUDE_DIRS "")
set(LIDAR_BASED_TRACKING_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers")

if(OpenCV_FOUND)
        set(LIDAR_BASED_TRACKING_SOURCES ${LIDAR_BASED_TRACKING_SOURCES} "LidarBasedTracking.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
        set(LIDAR_BASED_TRACKING_DEPENDENCIES ${LIDAR_BASED_TRACKING_DEPENDENCIES} opencv_core)
endif()

add_library(
    cdff_dfn_lidar_based_tracking
    ${LIDAR_BASED_TRACKING_SOURCES}
)
target_include_directories(
    cdff_dfn_lidar_based_tracking
    SYSTEM PRIVATE ${LIDAR_BASED_TRACKING_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_lidar_based_tracking
    ${LIDAR_BASED_TRACKING_DEPENDENCIES}
)
