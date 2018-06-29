add_executable(
    quality_registration_from_stereo
    Reconstruction3D/RegistrationFromStereo.cpp
    Reconstruction3D/ReconstructionExecutor.cpp
)
target_link_libraries(
	quality_registration_from_stereo
	registration_from_stereo converters_opencv cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui boost_system yaml-cpp dfpc_configurator
)

add_executable(
    quality_sparse_registration_from_stereo
    Reconstruction3D/SparseRegistrationFromStereo.cpp
    Reconstruction3D/ReconstructionExecutor.cpp
)
target_link_libraries(
	quality_sparse_registration_from_stereo
	sparse_registration_from_stereo converters_opencv cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui boost_system yaml-cpp dfpc_configurator
)

add_executable(
    quality_dense_registration_from_stereo
    Reconstruction3D/DenseRegistrationFromStereo.cpp
    Reconstruction3D/ReconstructionExecutor.cpp
)
target_link_libraries(
	quality_dense_registration_from_stereo
	dense_registration_from_stereo converters_opencv cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui boost_system yaml-cpp dfpc_configurator
)

add_executable(
    quality_adjustment_from_stereo
    Reconstruction3D/AdjustmentFromStereo.cpp
    Reconstruction3D/ReconstructionExecutor.cpp
)
target_link_libraries(
	quality_adjustment_from_stereo
	adjustment_from_stereo converters_opencv cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui boost_system yaml-cpp dfpc_configurator
)

add_executable(
    quality_reconstruction_from_stereo
    Reconstruction3D/ReconstructionFromStereo.cpp
    Reconstruction3D/ReconstructionExecutor.cpp
)
target_link_libraries(
	quality_reconstruction_from_stereo
	reconstruction_from_stereo converters_opencv cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui boost_system yaml-cpp dfpc_configurator
)

add_executable(
    quality_reconstruction_from_motion
    Reconstruction3D/ReconstructionFromMotion.cpp
    Reconstruction3D/ReconstructionExecutor.cpp
)
target_link_libraries(
	quality_reconstruction_from_motion
	reconstruction_from_motion converters_opencv cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui boost_system yaml-cpp dfpc_configurator
)

