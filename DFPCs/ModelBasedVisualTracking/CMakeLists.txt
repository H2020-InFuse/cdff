if(WITH_DLRTRACKER)
 add_library(
  cdff_dfpc_model_based_visual_tracking
  ModelBasedVisualTrackingInterface.cpp
 )
 add_library(
  cdff_dfpc_implementation_edge_model_contour_matching
  EdgeModelContourMatching.cpp
 )
 target_link_libraries(
   cdff_dfpc_implementation_edge_model_contour_matching
   PRIVATE cdff_dfpc_model_based_visual_tracking ${DLRTRACKER-CORE_LIBRARIES}
   PUBLIC cdff_types yaml-cpp cdff_helpers opencv_imgproc opencv_core opencv_imgcodecs cdff_visualizers_pcl dfpc_configurator 
 )
endif()





