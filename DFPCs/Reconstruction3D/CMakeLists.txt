set(RECONSTRUCTION_3D_SOURCES "Reconstruction3DInterface.cpp")
set(RECONSTRUCTION_3D_INCLUDE_DIRS "")
set(RECONSTRUCTION_3D_DEPENDENCIES "cdff_dfpc_configurator")

if(PCL_FOUND)
	set(RECONSTRUCTION_3D_SOURCES ${RECONSTRUCTION_3D_SOURCES} 
		"DenseRegistrationFromStereo.cpp" 
		"BundleHistory.cpp" 
		"PointCloudMap.cpp"
		"SparseRegistrationFromStereo.cpp"
		"ReconstructionFromMotion.cpp"
		"AdjustmentFromStereo.cpp"
		"EstimationFromStereo.cpp"
		"RegistrationFromStereo.cpp"
		"ReconstructionFromStereo.cpp"
		"ObservedScene.cpp"
		"MultipleCorrespondences2DRecorder.cpp"
		"MultipleCorrespondences3DRecorder.cpp"
		)
	set(RECONSTRUCTION_3D_INCLUDE_DIRS ${RECONSTRUCTION_3D_INCLUDE_DIRS} 
		${PCL_INCLUDE_DIRS}
		)
	set(RECONSTRUCTION_3D_DEPENDENCIES ${RECONSTRUCTION_3D_DEPENDENCIES}
		${PCL_COMMON_LIBRARIES} 
		${PCL_KDTREE_LIBRARIES}
		"cdff_dfn_dfnexecutors"
		"cdff_dfn_image_filtering"
		"cdff_dfn_stereo_reconstruction"
		"cdff_dfn_features_extraction_2d"
		"cdff_dfn_features_description_2d"
		"cdff_dfn_features_matching_2d"
		"cdff_dfn_bundle_adjustment"
		"cdff_dfn_fundamental_matrix_computation"
		"cdff_dfn_perspective_n_point_solving"
		"cdff_dfn_point_cloud_reconstruction_2d_to_3d"
		"cdff_dfn_features_extraction_3d"
		"cdff_dfn_features_description_3d"
		"cdff_dfn_features_matching_3d"
		"cdff_dfn_registration_3d"
		"cdff_dfn_point_cloud_assembly"
		"cdff_dfn_point_cloud_filtering"
		"cdff_dfn_point_cloud_transform"
		"cdff_dfn_transform_3d_estimation"
		"cdff_dfn_cameras_transform_estimation"
		)
endif()


add_library(
    cdff_dfpc_reconstruction_3d
    ${RECONSTRUCTION_3D_SOURCES}
    )
target_include_directories(
    cdff_dfpc_reconstruction_3d 
    SYSTEM PRIVATE ${RECONSTRUCTION_3D_INCLUDE_DIRS}
    )
target_link_libraries(
    cdff_dfpc_reconstruction_3d
    ${RECONSTRUCTION_3D_DEPENDENCIES}
    )
