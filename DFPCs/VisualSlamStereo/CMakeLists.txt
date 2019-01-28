set(VISUAL_SLAM_STEREO_SOURCES "VisualSlamStereoInterface.cpp")
set(VISUAL_SLAM_STEREO_INCLUDE_DIRS "")
set(VISUAL_SLAM_STEREO_DEPENDENCIES "cdff_dfpc_configurator")

if(OpenCV_FOUND)
  # OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
  set(STEREO_SLAM_DEPENDENCIES ${STEREO_SLAM_DEPENDENCIES} ${OpenCV_LIBS})

  # Look for Core/liborbslam library, build this DFPC if found
  if(WITH_LIBORBSLAM)
    find_package(LIBORBSLAM QUIET)
    if(LIBORBSLAM_FOUND)
      set(VISUAL_SLAM_STEREO_SOURCES ${VISUAL_SLAM_STEREO_SOURCES} "VisualSlamStereo.cpp")
      set(VISUAL_SLAM_STEREO_INCLUDE_DIRS ${VISUAL_SLAM_STEREO_INCLUDE_DIRS} ${STEREO_SLAM_INCLUDE_DIRS})
      set(VISUAL_SLAM_STEREO_DEPENDENCIES ${VISUAL_SLAM_STEREO_DEPENDENCIES}
		${STEREO_SLAM_DEPENDENCIES} 
		"cdff_dfn_dfnexecutors"
		"cdff_dfn_stereo_slam"
		)
    endif()
  endif()
endif()
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
