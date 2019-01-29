set(STEREO_SLAM_SOURCES "StereoSlamInterface.cpp")
set(STEREO_SLAM_INCLUDE_DIRS "")
set(STEREO_SLAM_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers")

# Look for OpenCV
if(OpenCV_FOUND)
  # OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
  set(STEREO_SLAM_DEPENDENCIES ${STEREO_SLAM_DEPENDENCIES} opencv_core opencv_features2d opencv_flann opencv_highgui opencv_imgproc opencv_calib3d)

  # Look for Core/liborbslam library, build this DFN if found
  if(WITH_LIBORBSLAM)
    find_package(LIBORBSLAM QUIET)
    if(LIBORBSLAM_FOUND)
      set(STEREO_SLAM_SOURCES ${STEREO_SLAM_SOURCES} "StereoSlamOrb.cpp")
      set(STEREO_SLAM_INCLUDE_DIRS ${STEREO_SLAM_INCLUDE_DIRS} ${LIBORBSLAM_INCLUDE_DIRS})
      set(STEREO_SLAM_DEPENDENCIES ${STEREO_SLAM_DEPENDENCIES} ${LIBORBSLAM_LIBRARIES})
    endif()
  endif()
endif()

add_library(
    cdff_dfn_stereo_slam
    ${STEREO_SLAM_SOURCES}
)
target_include_directories(
    cdff_dfn_stereo_slam
    SYSTEM PUBLIC ${STEREO_SLAM_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_stereo_slam
    ${STEREO_SLAM_DEPENDENCIES}
)

