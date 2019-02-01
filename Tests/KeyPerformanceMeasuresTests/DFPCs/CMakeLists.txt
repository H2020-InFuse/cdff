if(DLRTRACKER-CORE_FOUND AND OpenCV_FOUND)
	add_executable(
		validation_DLR_model_tracker
		ModelBasedVisualTracking/EdgeModelContourMatching.cpp
		ModelBasedVisualTracking/EdgeTrackerExecutor.cpp
	)
	target_link_libraries(
		validation_DLR_model_tracker
		PRIVATE cdff_dfpc_model_based_visual_tracking
		PUBLIC cdff_converters cdff_logger opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui boost_system
	)
endif()

add_executable(
    quality_registration_from_stereo
    Reconstruction3D/RegistrationFromStereo.cpp
    Reconstruction3D/ReconstructionExecutor.cpp
)
target_compile_definitions(quality_registration_from_stereo PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	quality_registration_from_stereo
	cdff_dfpc_reconstruction_3d
)

add_executable(
    quality_sparse_registration_from_stereo
    Reconstruction3D/SparseRegistrationFromStereo.cpp
    Reconstruction3D/ReconstructionExecutor.cpp
)
target_compile_definitions(quality_sparse_registration_from_stereo PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	quality_sparse_registration_from_stereo
	cdff_dfpc_reconstruction_3d
)

add_executable(
    quality_dense_registration_from_stereo
    Reconstruction3D/DenseRegistrationFromStereo.cpp
    Reconstruction3D/ReconstructionExecutor.cpp
)
target_compile_definitions(quality_dense_registration_from_stereo PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	quality_dense_registration_from_stereo
	cdff_dfpc_reconstruction_3d
)

add_executable(
    quality_adjustment_from_stereo
    Reconstruction3D/AdjustmentFromStereo.cpp
    Reconstruction3D/ReconstructionExecutor.cpp
)
target_compile_definitions(quality_adjustment_from_stereo PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	quality_adjustment_from_stereo
	cdff_dfpc_reconstruction_3d
)

add_executable(
    quality_estimation_from_stereo
    Reconstruction3D/EstimationFromStereo.cpp
    Reconstruction3D/ReconstructionExecutor.cpp
)
target_compile_definitions(quality_estimation_from_stereo PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	quality_estimation_from_stereo
	cdff_dfpc_reconstruction_3d
)

add_executable(
    quality_reconstruction_from_stereo
    Reconstruction3D/ReconstructionFromStereo.cpp
    Reconstruction3D/ReconstructionExecutor.cpp
)
target_compile_definitions(quality_reconstruction_from_stereo PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	quality_reconstruction_from_stereo
	cdff_dfpc_reconstruction_3d
)

add_executable(
    quality_reconstruction_from_motion
    Reconstruction3D/ReconstructionFromMotion.cpp
    Reconstruction3D/ReconstructionExecutor.cpp
)
target_compile_definitions(quality_reconstruction_from_motion PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	quality_reconstruction_from_motion
	cdff_dfpc_reconstruction_3d
)


add_executable(
    correctness_features_matching_3d
    PointCloudModelLocalisation/CorrectLocalizationTester.cpp
    PointCloudModelLocalisation/FeaturesMatching3D.cpp
)
target_compile_definitions(correctness_features_matching_3d PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
	correctness_features_matching_3d
	cdff_dfpc_point_cloud_model_localisation
        cdff_visualizers_pcl
)
