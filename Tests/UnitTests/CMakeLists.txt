add_definitions(-DTEST_CONFIGURATIONS="${TEST_CONFIGURATIONS}")
add_definitions(-DTEST_DATA="${TEST_DATA}")

add_executable(
    ParametersHelperTest
    Common/Helpers/ParametersHelper.cpp
)
target_link_libraries(
	ParametersHelperTest
	cdff_helpers cdff_logger
)
add_test(ParametersHelperTest ParametersHelperTest)

add_executable(
    ImageUndistortionTest
    DFNs/ImageFiltering/ImageUndistortion.cpp
)
target_link_libraries(
	ImageUndistortionTest
	mock cdff_logger cdff_types image_undistortion  converters_opencv opencv_core opencv_imgproc opencv_imgcodecs 
)
add_test(ImageUndistortionTest ImageUndistortionTest)


add_executable(
    HarrisDetector2DTest
    DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp
)
target_link_libraries(
	HarrisDetector2DTest
	mock cdff_logger cdff_types harris_detector_2d  converters_opencv opencv_core opencv_imgproc opencv_imgcodecs 
)
add_test(HarrisDetector2DTest HarrisDetector2DTest)

add_executable(
    OrbDetectorDescriptorTest
    DFNs/FeaturesExtraction2D/OrbDetectorDescriptor.cpp
)
target_link_libraries(
	OrbDetectorDescriptorTest
	mock cdff_logger cdff_types orb_detector_descriptor  converters_opencv opencv_core opencv_imgproc opencv_imgcodecs 
)
add_test(OrbDetectorDescriptorTest OrbDetectorDescriptorTest)

add_executable(
    OrbDescriptorTest
    DFNs/FeaturesDescription2D/OrbDescriptor.cpp
)
target_link_libraries(
	OrbDescriptorTest
	mock cdff_logger cdff_types orb_descriptor  converters_opencv opencv_core opencv_imgproc opencv_imgcodecs 
)
add_test(OrbDescriptorTest OrbDescriptorTest)

add_executable(
    IterativePnpSolverTest
    DFNs/PerspectiveNPointSolving/IterativePnpSolver.cpp
)
target_link_libraries(
	IterativePnpSolverTest
	mock cdff_logger cdff_types iterative_pnp_solver  converters_opencv opencv_core opencv_imgproc opencv_imgcodecs 
)
add_test(IterativePnpSolverTest IterativePnpSolverTest)

add_executable(
    FundamentalMatrixRansacTest
    DFNs/FundamentalMatrixComputation/FundamentalMatrixRansac.cpp
)
target_link_libraries(
	FundamentalMatrixRansacTest
	mock cdff_logger cdff_types fundamental_matrix_ransac  converters_opencv opencv_core opencv_imgproc opencv_imgcodecs synthetic_generators
)
add_test(FundamentalMatrixRansacTest FundamentalMatrixRansacTest)

add_executable(
    EssentialMatrixDecompositionTest
    DFNs/CamerasTransformEstimation/EssentialMatrixDecomposition.cpp
)
target_link_libraries(
	EssentialMatrixDecompositionTest
	mock cdff_logger cdff_types cdff_helpers essential_matrix_decomposition  converters_opencv opencv_core opencv_imgproc opencv_imgcodecs synthetic_generators
)
add_test(EssentialMatrixDecompositionTest EssentialMatrixDecompositionTest)

add_executable(
    FlannMatcherTest
    DFNs/FeaturesMatching2D/FlannMatcher.cpp
)
target_link_libraries(
	FlannMatcherTest
	mock cdff_logger cdff_types flann_matcher  converters_opencv opencv_core opencv_imgproc opencv_imgcodecs 
)
add_test(FlannMatcherTest FlannMatcherTest)

add_executable(
    TriangulationTest
    DFNs/PointCloudReconstruction2DTo3D/Triangulation.cpp
)
target_link_libraries(
	TriangulationTest
	mock cdff_logger cdff_types triangulation  converters_opencv opencv_core opencv_imgproc opencv_imgcodecs synthetic_generators
)
add_test(TriangulationTest TriangulationTest)

add_executable(
    DisparityMappingTest
    DFNs/StereoReconstruction/DisparityMapping.cpp
)
target_link_libraries(
	DisparityMappingTest
	mock cdff_logger cdff_types disparity_mapping  converters_opencv opencv_core opencv_imgproc opencv_imgcodecs synthetic_generators
)
add_test(DisparityMappingTest DisparityMappingTest)

add_executable(
    HarrisDetector3DTest
    DFNs/FeaturesExtraction3D/HarrisDetector3D.cpp
)
target_link_libraries(
	HarrisDetector3DTest
	mock cdff_logger cdff_types harris_detector_3d  converters_opencv opencv_core opencv_imgproc opencv_imgcodecs pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo 
)
add_test(HarrisDetector3DTest HarrisDetector3DTest)

add_executable(
    ShotDescriptor3DTest
    DFNs/FeaturesDescription3D/ShotDescriptor3D.cpp
)
target_link_libraries(
	ShotDescriptor3DTest
	mock cdff_logger cdff_types shot_descriptor_3d  converters_opencv opencv_core opencv_imgproc opencv_imgcodecs pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo 
)
add_test(ShotDescriptor3DTest ShotDescriptor3DTest)

add_executable(
    Ransac3DTest
    DFNs/FeaturesMatching3D/Ransac3D.cpp
)
target_link_libraries(
	Ransac3DTest
	mock cdff_logger cdff_types ransac_3d  converters_opencv opencv_core opencv_imgproc opencv_imgcodecs pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo 
)
add_test(Ransac3DTest Ransac3DTest)

add_executable(
    Icp3DTest
    DFNs/FeaturesMatching3D/Icp3D.cpp
)
target_link_libraries(
	Icp3DTest
	mock cdff_logger cdff_types icp_3d  converters_opencv opencv_core opencv_imgproc opencv_imgcodecs pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo 
)
add_test(Icp3DTest Icp3DTest)

add_executable(
    MatFrameConvertersTest
    Common/Converters/MatFrameConvertersTest.cpp
)
target_link_libraries(
	MatFrameConvertersTest cdff_types converters_opencv cdff_logger opencv_core opencv_imgproc opencv_imgcodecs 
)
add_test(MatFrameConvertersTest MatFrameConvertersTest)

add_executable(
    MatVisualPointFeatureVector2DConvertersTest
    Common/Converters/MatVisualPointFeatureVector2DConvertersTest.cpp
)
target_link_libraries(
	MatVisualPointFeatureVector2DConvertersTest cdff_types converters_opencv cdff_logger  opencv_core opencv_imgproc opencv_imgcodecs 
)
add_test(MatVisualPointFeatureVector2DConvertersTest
         MatVisualPointFeatureVector2DConvertersTest)

add_executable(
    MatVisualPointFeatureVector3DConvertersTest
    Common/Converters/MatVisualPointFeatureVector3DConvertersTest.cpp
)
target_link_libraries(
	MatVisualPointFeatureVector3DConvertersTest cdff_types converters_opencv cdff_logger  opencv_core opencv_imgproc opencv_imgcodecs
)
add_test(MatVisualPointFeatureVector3DConvertersTest
         MatVisualPointFeatureVector3DConvertersTest)

add_executable(
    PointCloudPclPointCloudConvertersTest
    Common/Converters/PointCloudPclPointCloudConvertersTest.cpp
)
target_link_libraries(
	PointCloudPclPointCloudConvertersTest cdff_types converters_opencv cdff_logger opencv_core opencv_imgproc opencv_imgcodecs pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo 
)
add_test(PointCloudPclPointCloudConvertersTest PointCloudPclPointCloudConvertersTest)

add_executable(
    PointCloudPclNormalsCloudConvertersTest
    Common/Converters/PointCloudPclNormalsCloudConvertersTest.cpp
)
target_link_libraries(
	PointCloudPclNormalsCloudConvertersTest cdff_types converters_opencv cdff_logger opencv_core opencv_imgproc opencv_imgcodecs pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo 
)
add_test(PointCloudPclNormalsCloudConvertersTest PointCloudPclNormalsCloudConvertersTest)

add_executable(
    VisualPointFeatureVector3DPclPointCloudConvertersTest
    Common/Converters/VisualPointFeatureVector3DPclPointCloudConvertersTest.cpp
)
target_link_libraries(
	VisualPointFeatureVector3DPclPointCloudConvertersTest cdff_types converters_opencv cdff_logger opencv_core opencv_imgproc opencv_imgcodecs pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo 
)
add_test(VisualPointFeatureVector3DPclPointCloudConvertersTest VisualPointFeatureVector3DPclPointCloudConvertersTest)

add_executable(
    Transform3DEigenTransformConvertersTest
    Common/Converters/Transform3DEigenTransformConvertersTest.cpp
)
target_link_libraries(
	Transform3DEigenTransformConvertersTest cdff_types converters_opencv cdff_logger
)
add_test(Transform3DEigenTransformConvertersTest Transform3DEigenTransformConvertersTest)

add_executable(
    Transform3DMatConvertersTest
    Common/Converters/Transform3DMatConvertersTest.cpp
)
target_link_libraries(
	Transform3DMatConvertersTest cdff_types converters_opencv cdff_logger
)
add_test(Transform3DMatConvertersTest Transform3DMatConvertersTest)


add_executable(
    DFNsChainInterfaceTest
    DFPCs/DFNsChainInterface.cpp
)
target_link_libraries(
	DFNsChainInterfaceTest
	cdff_helpers cdff_logger dfns_chain_interface
)
add_test(DFNsChainInterfaceTest DFNsChainInterfaceTest)


add_executable(
    StructureFromMotionTest
    DFPCs/PointCloudModelLocalisation/StructureFromMotion.cpp
)
target_link_libraries(
	StructureFromMotionTest
	cdff_helpers cdff_logger structure_from_motion opencv_highgui opencv_core opencv_imgproc opencv_imgcodecs mock stub_observed_scene boost_system
)
add_test(StructureFromMotionTest StructureFromMotionTest)

add_executable(
    ObservedSceneTest
    DFPCs/PointCloudModelLocalisation/ObservedScene.cpp
)
target_link_libraries(
	ObservedSceneTest
	cdff_helpers cdff_logger structure_from_motion opencv_highgui opencv_core opencv_imgproc opencv_imgcodecs mock stub_observed_scene
)
add_test(ObservedSceneTest ObservedSceneTest)
