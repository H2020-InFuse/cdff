add_executable(
    quality_registration_from_stereo
    Reconstruction3D/RegistrationFromStereo.cpp
    Reconstruction3D/ReconstructionExecutor.cpp
)
target_link_libraries(
	quality_registration_from_stereo
	registration_from_stereo mock converters_opencv cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui boost_system yaml-cpp dfpc_configurator
)

