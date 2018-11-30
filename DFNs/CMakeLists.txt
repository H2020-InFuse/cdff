MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})

FOREACH(subdir ${SUBDIRS})
  ADD_SUBDIRECTORY(${subdir})
ENDFOREACH()

add_library(
    dfns_builder
    DFNsBuilder.cpp
)
target_link_libraries(
  dfns_builder
  cdff_helpers
  cdff_logger
  cdff_types
  essential_matrix_decomposition
  fundamental_matrix_ransac
  orb_descriptor
  shot_descriptor_3d
  ransac_3d icp_3d
  best_descriptor_match
  harris_detector_2d
  cdff_dfn_image_filtering
  cdff_dfn_stereo_reconstruction
  orb_detector_descriptor
  flann_matcher
  iterative_pnp_solver
  triangulation
  harris_detector_3d
  iss_detector_3d
  corner_detector_3d
  kalman_predictor
  cdff_dfn_registration_3d
  ceres_adjustment
  svd_decomposition
  transform_3d_estimation_least_squares_minimization
  transform_3d_estimation_ceres
  hu_invariants
  octree
  cdff_converters
  pcl_common
  pcl_features
  pcl_search
  pcl_filters
  pcl_visualization
  pcl_io
  pcl_ml
  pcl_io_ply
  pcl_octree
  pcl_outofcore
  pcl_kdtree
  pcl_tracking
  pcl_stereo
  pcl_recognition
  pcl_registration
  pcl_people
  pcl_keypoints
  pcl_surface
  pcl_segmentation
  pcl_sample_consensus
  pcl_stereo
  convolution_filter
  threshold_force
  neighbour_point_average
  neighbour_single_point_average
  voxel_binning
  matcher_assembly
  cartesian_system_transform
  statistical_outlier_removal
)
