add_executable(
    ImageUndistortionGuiTest
    DFNs/ImageFiltering/ImageUndistortion.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(ImageUndistortionGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	ImageUndistortionGuiTest
	mock cdff_logger image_undistortion converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui
)

add_executable(
    ImageUndistortionRectificationGuiTest
    DFNs/ImageFiltering/ImageUndistortionRectification.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(ImageUndistortionRectificationGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	ImageUndistortionRectificationGuiTest
	mock cdff_logger image_undistortion_rectification converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui
)

add_executable(
    HarrisDetector2DGuiTest
    DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(HarrisDetector2DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	HarrisDetector2DGuiTest
	mock cdff_logger harris_detector_2d converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui
)

add_executable(
    OrbDetectorDescriptorGuiTest
    DFNs/FeaturesExtraction2D/OrbDetectorDescriptor.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(OrbDetectorDescriptorGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	OrbDetectorDescriptorGuiTest
	orb_detector_descriptor mock cdff_logger orb_detector_descriptor converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo boost_system yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    FlannMatcherGuiTest
    DFNs/FeaturesMatching2D/FlannMatcher.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(FlannMatcherGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	FlannMatcherGuiTest
	flann_matcher mock cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo boost_system yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    DisparityMappingGuiTest
    DFNs/StereoReconstruction/DisparityMapping.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(DisparityMappingGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	DisparityMappingGuiTest
	disparity_mapping mock cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui cdff_pcl_visualizer ${VTK_LIBRARIES}
)

add_executable(
    HirschmullerDisparityMappingGuiTest
    DFNs/StereoReconstruction/HirschmullerDisparityMapping.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(HirschmullerDisparityMappingGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	HirschmullerDisparityMappingGuiTest
	hirschmuller_disparity_mapping mock cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui cdff_pcl_visualizer ${VTK_LIBRARIES}
)

add_executable(
    ScanlineOptimizationGuiTest
    DFNs/StereoReconstruction/ScanlineOptimization.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(ScanlineOptimizationGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	ScanlineOptimizationGuiTest
	scanline_optimization mock cdff_logger shot_descriptor_3d converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui cdff_pcl_visualizer pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo boost_system yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    HarrisDetector3DGuiTest
    DFNs/FeaturesExtraction3D/HarrisDetector3D.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(HarrisDetector3DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	HarrisDetector3DGuiTest
	 harris_detector_3d mock cdff_logger harris_detector_2d converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui cdff_pcl_visualizer ${VTK_LIBRARIES}
)

add_executable(
    ShotDescriptor3DGuiTest
    DFNs/FeaturesDescription3D/ShotDescriptor3D.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(ShotDescriptor3DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	ShotDescriptor3DGuiTest
	shot_descriptor_3d mock cdff_logger shot_descriptor_3d converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo boost_system yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    Ransac3DGuiTest
    DFNs/FeaturesMatching3D/Ransac3D.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(Ransac3DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	Ransac3DGuiTest
	ransac_3d mock cdff_logger harris_detector_3d converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo boost_system yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    Icp3DGuiTest
    DFNs/FeaturesMatching3D/Icp3D.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(Icp3DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	Icp3DGuiTest
	icp_3d mock cdff_logger icp_3d converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo boost_system yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    TriangulationGuiTest
    DFNs/PointCloudReconstruction2DTo3D/Triangulation.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(TriangulationGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	TriangulationGuiTest
	triangulation mock cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo boost_system yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    StereoReconstruction
    DFNsIntegration/StereoReconstruction/StereoReconstruction.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNsIntegration/DFNsIntegrationTestInterface.cpp
)
set_target_properties(StereoReconstruction PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	StereoReconstruction
	image_undistortion orb_detector_descriptor triangulation flann_matcher fundamental_matrix_ransac essential_matrix_decomposition mock cdff_logger icp_3d converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo boost_system yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    StereoReconstruction2
    DFNsIntegration/StereoReconstruction/StereoReconstruction2.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNsIntegration/DFNsIntegrationTestInterface.cpp
)
set_target_properties(StereoReconstruction2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	StereoReconstruction2
	image_undistortion harris_detector_2d orb_descriptor triangulation flann_matcher fundamental_matrix_ransac essential_matrix_decomposition mock cdff_logger icp_3d converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo boost_system yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    FeaturesMatching3DGuiTest
    DFPCs/PointCloudModelLocalisation/FeaturesMatching3D.cpp
)
set_target_properties(FeaturesMatching3DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	FeaturesMatching3DGuiTest
	dfpc_implementation_features_matching_3d mock converters_opencv cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui boost_system yaml-cpp dfpc_configurator  ${VTK_LIBRARIES}
)

add_executable(
    ReconstructionFromMotionGuiTest
    DFPCs/Reconstruction3D/ReconstructionFromMotion.cpp
    DFPCs/Reconstruction3D/GuiTestReconstruction3D.cpp
)
set_target_properties(ReconstructionFromMotionGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	ReconstructionFromMotionGuiTest
	reconstruction_from_motion mock converters_opencv cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui boost_system yaml-cpp dfpc_configurator ${VTK_LIBRARIES}
)

add_executable(
    ReconstructionFromStereoGuiTest
    DFPCs/Reconstruction3D/ReconstructionFromStereo.cpp
    DFPCs/Reconstruction3D/GuiTestReconstruction3D.cpp
)
set_target_properties(ReconstructionFromStereoGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	ReconstructionFromStereoGuiTest
	reconstruction_from_stereo mock converters_opencv cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui boost_system yaml-cpp dfpc_configurator ${VTK_LIBRARIES}
)

add_executable(
    RegistrationFromStereoGuiTest
    DFPCs/Reconstruction3D/RegistrationFromStereo.cpp
    DFPCs/Reconstruction3D/GuiTestReconstruction3D.cpp
)
set_target_properties(RegistrationFromStereoGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	RegistrationFromStereoGuiTest
	registration_from_stereo mock converters_opencv cdff_logger converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui boost_system yaml-cpp dfpc_configurator ${VTK_LIBRARIES}
)

