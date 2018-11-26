add_definitions(-DTEST_CONFIGURATIONS="${TEST_CONFIGURATIONS}")
add_definitions(-DTEST_DATA="${TEST_DATA}")

# Path to the directory containing Catch's single-include header (catch.hpp)
set(CATCH_INCLUDE_DIR "Catch")

# Define the Catch library
add_library(catch INTERFACE)
target_include_directories(catch SYSTEM INTERFACE "${CATCH_INCLUDE_DIR}")

# Source files containing Catch test cases. They will be linked together to
# produce the test executable. Add any additional file in ALPHABETICAL ORDER.
set(unittest_sources UnitTests.cpp
    Common/BitStream/BitStream.cpp
    Common/Converters/MatCorrespondenceMaps2DSequenceConvertersTest.cpp
    Common/Converters/MatCorrespondenceMaps3DSequenceConvertersTest.cpp
    Common/Converters/MatFrameConvertersTest.cpp
    Common/Converters/MatVisualPointFeatureVector2DConvertersTest.cpp
    Common/Converters/MatVisualPointFeatureVector3DConvertersTest.cpp
    Common/Converters/PointCloudPclNormalsCloudConvertersTest.cpp
    Common/Converters/PointCloudPclPointCloudConvertersTest.cpp
    Common/Converters/Transform3DEigenTransformConvertersTest.cpp
    Common/Converters/Transform3DMatConvertersTest.cpp
    Common/Converters/VisualPointFeatureVector3DPclPointCloudConvertersTest.cpp
    Common/Helpers/ParametersHelper.cpp
    Common/Types/CorrespondenceMap2D.cpp
    DFNs/BundleAdjustment/CeresAdjustment.cpp
    DFNs/BundleAdjustment/SvdDecomposition.cpp
    DFNs/CamerasTransformEstimation/EssentialMatrixDecomposition.cpp
    DFNs/DepthFiltering/DepthFiltering.cpp
    DFNs/FeaturesDescription2D/OrbDescriptor.cpp
    DFNs/FeaturesDescription3D/ShotDescriptor3D.cpp
    DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp
    DFNs/FeaturesExtraction2D/OrbDetectorDescriptor.cpp
    DFNs/FeaturesExtraction3D/HarrisDetector3D.cpp
    DFNs/FeaturesExtraction3D/IssDetector3D.cpp
    DFNs/FeaturesMatching2D/FlannMatcher.cpp
    DFNs/FeaturesMatching3D/BestDescriptorMatch.cpp
    DFNs/FeaturesMatching3D/Icp3D.cpp
    DFNs/FeaturesMatching3D/Ransac3D.cpp
    DFNs/ForceMeshGenerator/ForceMeshGenerator.cpp
    DFNs/ForceMeshGenerator/ForceMeshHelperFunctions.cpp
    DFNs/FundamentalMatrixComputation/FundamentalMatrixRansac.cpp
    DFNs/ImageFiltering/ImageUndistortion.cpp
    DFNs/KFCorrection/KalmanCorrector.cpp
    DFNs/KFPrediction/KalmanPredictor.cpp
    DFNs/PerspectiveNPointSolving/IterativePnpSolver.cpp
    DFNs/PointCloudAssembly/NeighbourPointAverage.cpp
    DFNs/PointCloudAssembly/NeighbourSinglePointAverage.cpp
    DFNs/PointCloudAssembly/VoxelBinning.cpp
    #DFNs/PointCloudAssembly/MatcherAssembly.cpp
    DFNs/PointCloudFiltering/StatisticalOutlierRemoval.cpp
    DFNs/PointCloudReconstruction2DTo3D/Triangulation.cpp
    DFNs/PointCloudTransform/CartesianSystemTransform.cpp
    DFNs/PrimitiveMatching/HuInvariants.cpp
    DFNs/Registration3D/Icp3D.cpp
    DFNs/Registration3D/IcpCC.cpp
    #DFNs/Registration3D/IcpMatcher.cpp
    DFNs/StereoReconstruction/DisparityMapping.cpp
    DFNs/StereoReconstruction/HirschmullerDisparityMapping.cpp
    DFNs/Transform3DEstimation/CeresEstimation.cpp
    DFNs/Transform3DEstimation/LeastSquaresMinimization.cpp
    DFNs/Voxelization/Octree.cpp
    DFPCs/ModelBasedVisualTracking/EdgeModelContourMatching.cpp # memory intensive!
    DFPCs/DfpcConfigurator.cpp
    DFPCs/HapticScanning/HapticScanning.cpp
    DFPCs/PointCloudModelLocalisation/FeaturesMatching3D.cpp
    DFPCs/Reconstruction3D/BundleHistory.cpp
    DFPCs/Reconstruction3D/MultipleCorrespondences2DRecorder.cpp
    DFPCs/Reconstruction3D/MultipleCorrespondencesRecorder.cpp
    DFPCs/Reconstruction3D/PointCloudMap.cpp
)

# Libraries used by the aforementioned source files. They will be linked in the
# test executable. Add any additional library in ALPHABETICAL ORDER, and ONLY
# list the necessary ones.
set(unittest_libraries catch
    adjustment_from_stereo
    best_descriptor_match
    cartesian_system_transform
    cc_core_lib
    cdff_converters
    cdff_dfn_dfnexecutors
    cdff_helpers
    cdff_logger
    cdff_types
    ceres_adjustment
    convolution_filter
    dfpc_configurator
    cdff_dfpc_implementation_edge_model_contour_matching
    dfpc_haptic_scanning
    dfpc_implementation_features_matching_3d
    disparity_mapping
    essential_matrix_decomposition
    estimation_from_stereo
    flann_matcher
    fundamental_matrix_ransac
    harris_detector_2d
    harris_detector_3d
    iss_detector_3d
    hirschmuller_disparity_mapping
    hu_invariants
    icp_3d
    image_undistortion
    iterative_pnp_solver
    kalman_corrector
    kalman_predictor
#[=[mock]=] # no longer used by any DFN unit test
    neighbour_point_average
    neighbour_single_point_average
    opencv_core
    opencv_highgui
    opencv_imgcodecs
    opencv_imgproc
    opencv_video
    orb_descriptor
    orb_detector_descriptor
    pcl_common
    pcl_features
    pcl_filters
    pcl_io
    pcl_io_ply
    pcl_kdtree
    pcl_keypoints
    pcl_ml
    pcl_octree
    pcl_outofcore
    pcl_people
    pcl_recognition
    pcl_registration
    pcl_sample_consensus
    pcl_search
    pcl_segmentation
    pcl_stereo
    pcl_surface
    pcl_tracking
    pcl_visualization
    ransac_3d
#[=[reconstruction_from_stereo]=]
    registration_icp_3d
    registration_icp_cc
    registration_icp_matcher
    shot_descriptor_3d
    statistical_outlier_removal
    svd_decomposition
    synthetic_generators
    threshold_force
    transform_3d_estimation_ceres
    transform_3d_estimation_least_squares_minimization
    triangulation
    voxel_binning
    matcher_assembly
)

# Make the Catch executable that runs the unit tests
add_executable(cdff-unit-tests ${unittest_sources})
target_compile_definitions(cdff-unit-tests PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(cdff-unit-tests ${unittest_libraries})

# Register the Catch executable as a CTest test
#
# Working directory for this test: if WORKING_DIRECTORY is not specified the
# test will be run with the current working directory set to the build directory
# corresponding to the current source directory (i.e. corresponding to
# ${CMAKE_CURRENT_SOURCE_DIR}).
add_test(NAME unit-tests COMMAND cdff-unit-tests)
