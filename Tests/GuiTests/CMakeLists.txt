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
    cdff_dfn_features_extraction_2d
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
    cdff_dfn_features_extraction_2d
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
        cdff_dfn_depth_filtering
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
    cdff_dfn_features_matching_2d
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
    cdff_dfn_image_filtering
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
    cdff_dfn_image_filtering
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
        cdff_dfn_image_filtering
        ${GRAPHICALTEST_CDFF_LIBRARIES}
        ${GRAPHICALTEST_OPENCV_LIBRARIES}
)
set_target_properties(kmeans_gui_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)

add_executable(
    edge_detection_gui_test
    DFNs/ImageFiltering/CannyEdgeDetection.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
target_link_libraries(
    edge_detection_gui_test
    cdff_dfn_image_filtering
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
    cdff_dfn_image_filtering
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
    cdff_dfn_features_extraction_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES} cdff_visualizers_pcl
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
)

add_executable(
    IssDetector3DGuiTest
    DFNs/FeaturesExtraction3D/IssDetector3D.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(IssDetector3DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(IssDetector3DGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    IssDetector3DGuiTest
    cdff_dfn_features_extraction_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES} cdff_visualizers_pcl
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
)

add_executable(
    CornerDetector3DGuiTest
    DFNs/FeaturesExtraction3D/CornerDetector3D.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(CornerDetector3DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(CornerDetector3DGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    CornerDetector3DGuiTest
    cdff_dfn_features_extraction_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES} cdff_visualizers_pcl
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
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
    cdff_dfn_registration_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES} cdff_visualizers_pcl
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
    cdff_dfn_registration_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES} cdff_visualizers_pcl
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
    cdff_dfn_features_description_3d
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
    cdff_dfn_features_matching_3d
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
    cdff_dfn_features_matching_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_io_ply pcl_octree pcl_kdtree pcl_registration pcl_keypoints pcl_sample_consensus
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
    cdff_dfn_point_cloud_reconstruction_2d_to_3d
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
    cdff_dfn_stereo_reconstruction
    ${GRAPHICALTEST_CDFF_LIBRARIES} cdff_visualizers_pcl
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
    cdff_dfn_stereo_reconstruction
    ${GRAPHICALTEST_CDFF_LIBRARIES} cdff_visualizers_pcl
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
    cdff_dfn_stereo_reconstruction
    ${GRAPHICALTEST_CDFF_LIBRARIES} cdff_visualizers_pcl
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
        cdff_dfn_primitive_matching
        ${GRAPHICALTEST_CDFF_LIBRARIES}
        ${GRAPHICALTEST_OPENCV_LIBRARIES}
)

add_executable(
    ColorConversionGuiTest
    DFNs/ColorConversion/ColorConversion.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(ColorConversionGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(ColorConversionGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    ColorConversionGuiTest
    cdff_dfn_color_conversion
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
)

add_executable(
    DisparityImageGuiTest
    DFNs/DisparityImage/DisparityImage.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(DisparityImageGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(DisparityImageGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    DisparityImageGuiTest
    cdff_dfn_disparity_image
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
)

if(WITH_EDRES)
add_executable(
    DisparityImageEdresGuiTest
    DFNs/DisparityImage/DisparityImageEdres.cpp
    DFNs/DFNTestInterface.cpp
    ${GRAPHICALTEST_GUI_SOURCES}
)
set_target_properties(DisparityImageEdresGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_compile_definitions(DisparityImageEdresGuiTest PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
    DisparityImageEdresGuiTest
    cdff_dfn_disparity_image
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
)
endif()

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
    cdff_dfpc_reconstruction_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
    cdff_visualizers_pcl
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
    cdff_dfpc_reconstruction_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
    cdff_visualizers_pcl
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
    cdff_dfpc_reconstruction_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
    cdff_visualizers_pcl
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
    cdff_dfpc_reconstruction_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
    cdff_visualizers_pcl
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
    cdff_dfpc_reconstruction_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
    cdff_visualizers_pcl
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
    cdff_dfpc_reconstruction_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
    cdff_visualizers_pcl
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
    cdff_dfpc_reconstruction_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
    cdff_visualizers_pcl
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
    cdff_dfpc_point_cloud_model_localisation
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
    cdff_visualizers_pcl
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
    cdff_dfpc_reconstruction_and_identification
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    ${VTK_LIBRARIES}
    cdff_visualizers_pcl
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
    cdff_dfn_image_filtering cdff_dfn_features_extraction_2d cdff_dfn_point_cloud_reconstruction_2d_to_3d cdff_dfn_features_matching_2d cdff_dfn_fundamental_matrix_computation cdff_dfn_cameras_transform_estimation cdff_dfn_features_matching_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_io_ply pcl_octree pcl_kdtree pcl_registration pcl_keypoints pcl_sample_consensus
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
    cdff_dfn_image_filtering cdff_dfn_features_extraction_2d cdff_dfn_features_description_2d cdff_dfn_point_cloud_reconstruction_2d_to_3d cdff_dfn_features_matching_2d cdff_dfn_fundamental_matrix_computation cdff_dfn_cameras_transform_estimation cdff_dfn_features_matching_3d
    ${GRAPHICALTEST_CDFF_LIBRARIES}
    ${GRAPHICALTEST_OPENCV_LIBRARIES}
    pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_io_ply pcl_octree pcl_kdtree pcl_registration pcl_keypoints pcl_sample_consensus
    yaml-cpp ${VTK_LIBRARIES}
)
