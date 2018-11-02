set(GRAPHICALTEST_GUI_SOURCES MainInterface.cpp ButtonsInterface.cpp ParametersInterface.cpp)

set(GRAPHICALTEST_CDFF_LIBRARIES cdff_converters cdff_logger)
set(GRAPHICALTEST_OPENCV_LIBRARIES opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui)

## Graphical tests for the DFNs ------------------------------------------------

add_executable(
    HarrisDetector2DGuiTest
    DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(HarrisDetector2DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
    HarrisDetector2DGuiTest
    harris_detector_2d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
)

add_executable(
    OrbDetectorDescriptorGuiTest
    DFNs/FeaturesExtraction2D/OrbDetectorDescriptor.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(OrbDetectorDescriptorGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
    OrbDetectorDescriptorGuiTest
    orb_detector_descriptor
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
)



add_executable(
        DepthFilteringGuiTest
        DFNs/DepthFiltering/DepthFiltering.cpp
        DFNs/DFNTestInterface.cpp
        ${GRAPHICALTEST_GUI_SOURCES}
)

set_target_properties(DepthFilteringGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
        DepthFilteringGuiTest
        convolution_filter
        ${GRAPHICALTEST_CDFF_LIBRARIES}
        ${GRAPHICALTEST_OPENCV_LIBRARIES}
)




add_executable(
    FlannMatcherGuiTest
    DFNs/FeaturesMatching2D/FlannMatcher.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(FlannMatcherGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
    FlannMatcherGuiTest
    flann_matcher
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
)

add_executable(
    ImageUndistortionGuiTest
    DFNs/ImageFiltering/ImageUndistortion.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(ImageUndistortionGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
    ImageUndistortionGuiTest
    image_undistortion
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
)

add_executable(
    ImageUndistortionRectificationGuiTest
    DFNs/ImageFiltering/ImageUndistortionRectification.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(ImageUndistortionRectificationGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
    ImageUndistortionRectificationGuiTest
    image_undistortion_rectification
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
)

add_executable(kmeans_gui_test
        DFNs/DFNTestInterface.cpp
        DFNs/ImageFiltering/KMeansClustering.cpp
        DFNs/DFNTestInterface.cpp
        ${GRAPHICALTEST_GUI_SOURCES}
)
target_link_libraries(
        kmeans_gui_test
        cdff_dfn_kmeans
        ${GRAPHICALTEST_CDFF_LIBRARIES}
        ${GRAPHICALTEST_OPENCV_LIBRARIES}
)
set_target_properties(kmeans_gui_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)

add_executable(
    edge_detection_gui_test
    DFNs/ImageFiltering/EdgeDetection.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
target_link_libraries(
    edge_detection_gui_test
    cdff_dfn_edge_detection
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${GRAPHICALTEST_CDFF_LIBRARIES}
)
set_target_properties(edge_detection_gui_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)

add_executable(
    normal_vector_extraction_gui_test
    DFNs/DFNTestInterface.cpp
    DFNs/ImageFiltering/NormalVectorExtraction.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
target_link_libraries(
    normal_vector_extraction_gui_test
    cdff_dfn_normal_vector_extraction
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
)
set_target_properties(normal_vector_extraction_gui_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)

add_executable(
    HarrisDetector3DGuiTest
    DFNs/FeaturesExtraction3D/HarrisDetector3D.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(HarrisDetector3DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(HarrisDetector3DGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    HarrisDetector3DGuiTest
    harris_detector_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES} cdff_pcl_visualizer
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
)

add_executable(
    Registration3DIcpGuiTest
    DFNs/Registration3D/Icp3D.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(Registration3DIcpGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(Registration3DIcpGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    Registration3DIcpGuiTest
    registration_icp_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES} cdff_pcl_visualizer
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_io_ply pcl_octree
    yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    Registration3DIcpCCGuiTest
    DFNs/Registration3D/IcpCC.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(Registration3DIcpCCGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(Registration3DIcpCCGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    Registration3DIcpCCGuiTest
    registration_icp_cc
    ${GRAPHICALTEST_CDFF_LIBRARIES} cdff_pcl_visualizer
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_io_ply pcl_octree
    yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    ShotDescriptor3DGuiTest
    DFNs/FeaturesDescription3D/ShotDescriptor3D.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(ShotDescriptor3DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(ShotDescriptor3DGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    ShotDescriptor3DGuiTest
    shot_descriptor_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_io_ply pcl_octree pcl_kdtree pcl_keypoints
    yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    Icp3DGuiTest
    DFNs/FeaturesMatching3D/Icp3D.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(Icp3DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(Icp3DGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    Icp3DGuiTest
    icp_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_io_ply pcl_octree pcl_kdtree pcl_registration pcl_keypoints pcl_sample_consensus
    yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    Ransac3DGuiTest
    DFNs/FeaturesMatching3D/Ransac3D.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(Ransac3DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(Ransac3DGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    Ransac3DGuiTest
    ransac_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_kdtree pcl_registration pcl_keypoints pcl_sample_consensus
    yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    TriangulationGuiTest
    DFNs/PointCloudReconstruction2DTo3D/Triangulation.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(TriangulationGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(TriangulationGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    TriangulationGuiTest
    triangulation
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    pcl_common pcl_visualization
    yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    DisparityMappingGuiTest
    DFNs/StereoReconstruction/DisparityMapping.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(DisparityMappingGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(DisparityMappingGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    DisparityMappingGuiTest
    disparity_mapping
    ${GRAPHICALTEST_CDFF_LIBRARIES} cdff_pcl_visualizer
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
)

add_executable(
    HirschmullerDisparityMappingGuiTest
    DFNs/StereoReconstruction/HirschmullerDisparityMapping.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(HirschmullerDisparityMappingGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(HirschmullerDisparityMappingGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    HirschmullerDisparityMappingGuiTest
    hirschmuller_disparity_mapping
    ${GRAPHICALTEST_CDFF_LIBRARIES} cdff_pcl_visualizer
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
)

add_executable(
    ScanlineOptimizationGuiTest
    DFNs/StereoReconstruction/ScanlineOptimization.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(ScanlineOptimizationGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(ScanlineOptimizationGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    ScanlineOptimizationGuiTest
    scanline_optimization
    ${GRAPHICALTEST_CDFF_LIBRARIES} cdff_pcl_visualizer
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    pcl_common pcl_visualization
    ${VTK_LIBRARIES}
)

add_executable(
        HuInvariantsGuiTest
        DFNs/PrimitiveMatching/HuInvariants.cpp
        DFNs/DFNTestInterface.cpp
        ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(HuInvariantsGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
        HuInvariantsGuiTest
        hu_invariants
        ${GRAPHICALTEST_CDFF_LIBRARIES}
        ${GRAPHICALTEST_OPENCV_LIBRARIES}
)


## Graphical tests for the DFPCs -----------------------------------------------

add_executable(
    DenseRegistrationFromStereoGuiTest
    DFPCs/Reconstruction3D/DenseRegistrationFromStereo.cpp
    DFPCs/Reconstruction3D/GuiTestReconstruction3D.cpp
)
set_target_properties(DenseRegistrationFromStereoGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(DenseRegistrationFromStereoGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    DenseRegistrationFromStereoGuiTest
    dense_registration_from_stereo
    ${GRAPHICALTEST_CDFF_LIBRARIES} dfpc_configurator  dfn_executors
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    SparseRegistrationFromStereoGuiTest
    DFPCs/Reconstruction3D/SparseRegistrationFromStereo.cpp
    DFPCs/Reconstruction3D/GuiTestReconstruction3D.cpp
)
set_target_properties(SparseRegistrationFromStereoGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(SparseRegistrationFromStereoGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    SparseRegistrationFromStereoGuiTest
    sparse_registration_from_stereo
    ${GRAPHICALTEST_CDFF_LIBRARIES} dfpc_configurator dfn_executors
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    AdjustmentFromStereoGuiTest
    DFPCs/Reconstruction3D/AdjustmentFromStereo.cpp
    DFPCs/Reconstruction3D/GuiTestReconstruction3D.cpp
)
set_target_properties(AdjustmentFromStereoGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(AdjustmentFromStereoGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    AdjustmentFromStereoGuiTest
    adjustment_from_stereo
    ${GRAPHICALTEST_CDFF_LIBRARIES} dfpc_configurator dfn_executors
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    ReconstructionFromMotionGuiTest
    DFPCs/Reconstruction3D/ReconstructionFromMotion.cpp
    DFPCs/Reconstruction3D/GuiTestReconstruction3D.cpp
)
set_target_properties(ReconstructionFromMotionGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(ReconstructionFromMotionGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    ReconstructionFromMotionGuiTest
    reconstruction_from_motion
    ${GRAPHICALTEST_CDFF_LIBRARIES} dfpc_configurator
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    ReconstructionFromStereoGuiTest
    DFPCs/Reconstruction3D/ReconstructionFromStereo.cpp
    DFPCs/Reconstruction3D/GuiTestReconstruction3D.cpp
)
set_target_properties(ReconstructionFromStereoGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(ReconstructionFromStereoGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    ReconstructionFromStereoGuiTest
    reconstruction_from_stereo
    ${GRAPHICALTEST_CDFF_LIBRARIES} dfpc_configurator dfn_executors
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    RegistrationFromStereoGuiTest
    DFPCs/Reconstruction3D/RegistrationFromStereo.cpp
    DFPCs/Reconstruction3D/GuiTestReconstruction3D.cpp
)
set_target_properties(RegistrationFromStereoGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(RegistrationFromStereoGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    RegistrationFromStereoGuiTest
    registration_from_stereo
    ${GRAPHICALTEST_CDFF_LIBRARIES} dfpc_configurator  dfn_executors
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    EstimationFromStereoGuiTest
    DFPCs/Reconstruction3D/EstimationFromStereo.cpp
    DFPCs/Reconstruction3D/GuiTestReconstruction3D.cpp
)
set_target_properties(EstimationFromStereoGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(EstimationFromStereoGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    EstimationFromStereoGuiTest
    estimation_from_stereo
    ${GRAPHICALTEST_CDFF_LIBRARIES} dfpc_configurator dfn_executors
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    FeaturesMatching3DGuiTest
    DFPCs/PointCloudModelLocalisation/FeaturesMatching3D.cpp
    DFPCs/PointCloudModelLocalisation/GuiTestPointCloudModelLocalisation.cpp
)
set_target_properties(FeaturesMatching3DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(FeaturesMatching3DGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    FeaturesMatching3DGuiTest
    dfpc_implementation_features_matching_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES} dfpc_configurator
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    RegistrationAndMatchingGuiTest
    DFPCs/ReconstructionAndIdentification/RegistrationAndMatching.cpp
    DFPCs/ReconstructionAndIdentification/GuiTestReconstructionAndIdentification.cpp
)
set_target_properties(RegistrationAndMatchingGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(RegistrationAndMatchingGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    RegistrationAndMatchingGuiTest
    registration_and_matching
    ${GRAPHICALTEST_CDFF_LIBRARIES} dfpc_configurator mock
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    yaml-cpp ${VTK_LIBRARIES}
)


add_executable(
    StereoReconstruction
    DFNsIntegration/StereoReconstruction/StereoReconstruction.cpp
    DFNsIntegration/DFNsIntegrationTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(StereoReconstruction PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(StereoReconstruction PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    StereoReconstruction
    image_undistortion orb_detector_descriptor triangulation flann_matcher fundamental_matrix_ransac essential_matrix_decomposition icp_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_kdtree pcl_registration pcl_keypoints pcl_sample_consensus
    yaml-cpp ${VTK_LIBRARIES}
)

add_executable(
    StereoReconstruction2
    DFNsIntegration/StereoReconstruction/StereoReconstruction2.cpp
    DFNsIntegration/DFNsIntegrationTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(StereoReconstruction2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(StereoReconstruction2 PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    StereoReconstruction2
    image_undistortion harris_detector_2d orb_descriptor triangulation flann_matcher fundamental_matrix_ransac essential_matrix_decomposition icp_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_kdtree pcl_registration pcl_keypoints pcl_sample_consensus
    yaml-cpp ${VTK_LIBRARIES}
)
