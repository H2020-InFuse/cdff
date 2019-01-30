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
  cdff_dfn_cameras_transform_estimation
  cdff_dfn_color_conversion
  cdff_dfn_disparity_image
  cdff_dfn_fundamental_matrix_computation
  cdff_dfn_features_description_2d
  cdff_dfn_image_degradation
  cdff_dfn_image_filtering
  cdff_dfn_image_rectification
  cdff_dfn_stereo_degradation
  cdff_dfn_stereo_reconstruction
  cdff_dfn_stereo_rectification
  cdff_dfn_features_extraction_2d
  cdff_dfn_features_matching_2d
  cdff_dfn_perspective_n_point_solving
  cdff_dfn_point_cloud_reconstruction_2d_to_3d
  cdff_dfn_kf_prediction
  cdff_dfn_kf_correction
  cdff_dfn_linemod_detect
  cdff_dfn_linemod_train
  cdff_dfn_registration_3d
  cdff_dfn_transform_3d_estimation
  cdff_dfn_primitive_matching
  cdff_dfn_voxelization
  cdff_converters
  cdff_dfn_depth_filtering
  cdff_dfn_force_mesh_generator
  cdff_dfn_point_cloud_assembly
  cdff_dfn_point_cloud_transformation
  cdff_dfn_point_cloud_filtering
  cdff_dfn_stereo_motion_estimation
  cdff_dfn_stereo_reconstruction
  cdff_dfn_features_extraction_3d
  cdff_dfn_features_description_3d
  cdff_dfn_features_matching_3d
  cdff_dfn_bundle_adjustment
  cdff_dfn_stereo_slam
  cdff_dfn_disparity_to_pointcloud
  cdff_dfn_disparity_to_pointcloud_with_intensity
  cdff_dfn_lidar_based_tracking
)
