if(WITH_DLRTRACKER)
 add_library(
  model_based_visual_tracking
  ModelBasedVisualTrackingInterface.cpp
 )
 target_link_libraries(
  model_based_visual_tracking
 )
 add_library(
  dfpc_implementation_edge_model_contour_matching
  EdgeModelContourMatching.cpp
 )
 target_link_libraries(
  dfpc_implementation_edge_model_contour_matching
  model_based_visual_tracking cdff_types yaml-cpp cdff_helpers opencv_imgproc opencv_core opencv_imgcodecs cdff_visualizers_pcl dfpc_configurator ${DLRTRACKER-CORE_LIBRARIES}
 )
endif()





