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
    DFNs/ColorConversion/ColorConversion.cpp
    DFNs/ImageFiltering/DerivativeEdgeDetection.cpp
    DFNs/ImageFiltering/CannyEdgeDetection.cpp
    DFNs/ImageFiltering/BackgroundExtraction.cpp
    DFNs/DepthFiltering/DepthFiltering.cpp
    DFNs/FeaturesDescription2D/OrbDescriptor.cpp
    DFNs/FeaturesDescription3D/ShotDescriptor3D.cpp
    DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp
    DFNs/FeaturesExtraction2D/OrbDetectorDescriptor.cpp
    DFNs/FeaturesExtraction3D/HarrisDetector3D.cpp
    DFNs/FeaturesExtraction3D/IssDetector3D.cpp
    DFNs/FeaturesExtraction3D/CornerDetector3D.cpp
    DFNs/FeaturesMatching2D/FlannMatcher.cpp
    DFNs/FeaturesMatching3D/BestDescriptorMatch.cpp
    DFNs/FeaturesMatching3D/Icp3D.cpp
    DFNs/FeaturesMatching3D/Ransac3D.cpp
    DFNs/FundamentalMatrixComputation/FundamentalMatrixRansac.cpp
    DFNs/ImageFiltering/ImageUndistortion.cpp
    DFNs/ImageFiltering/NormalVectorExtraction.cpp
    DFNs/ImageFiltering/KMeansClustering.cpp
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
    DFNs/DisparityToPointCloudWithIntensity/DisparityToPointCloudWithIntensity.cpp
    DFPCs/DfpcConfigurator.cpp
    DFPCs/HapticScanning/HapticScanning.cpp
    DFPCs/PointCloudModelLocalisation/FeaturesMatching3D.cpp
    DFPCs/Reconstruction3D/BundleHistory.cpp
    DFPCs/Reconstruction3D/MultipleCorrespondences2DRecorder.cpp
    DFPCs/Reconstruction3D/MultipleCorrespondencesRecorder.cpp
    DFPCs/Reconstruction3D/PointCloudMap.cpp
)

if(WITH_EDRES)
    set(unittest_sources ${unittest_sources}
        DFNs/DisparityToPointCloudWithIntensity/DisparityToPointCloudWithIntensityEdres.cpp
    )
endif()

# Libraries used by the aforementioned source files. They will be linked in the
# test executable. Add any additional library in ALPHABETICAL ORDER, and ONLY
# list the necessary ones.
set(unittest_libraries catch
    cdff_converters
    cdff_helpers
    cdff_logger
    cdff_types
    cdff_dfn_dfnexecutors
    cdff_dfn_point_cloud_transform
    cdff_dfn_depth_filtering
    cdff_dfn_cameras_transform_estimation
    cdff_dfn_color_conversion
    cdff_dfn_features_matching_2d
    cdff_dfn_fundamental_matrix_computation
    cdff_dfn_primitive_matching
    cdff_dfn_perspective_n_point_solving
    cdff_dfn_kf_correction
    cdff_dfn_kf_prediction
    cdff_dfn_point_cloud_assembly
    cdff_dfn_features_description_2d
    cdff_dfn_features_extraction_2d
    cdff_dfn_registration_3d
    cdff_dfn_image_filtering
    cdff_dfn_point_cloud_filtering
    cdff_dfn_transform_3d_estimation
    cdff_dfn_point_cloud_reconstruction_2d_to_3d
    cdff_dfn_stereo_reconstruction
    cdff_dfn_features_extraction_3d
    cdff_dfn_features_description_3d
    cdff_dfn_features_matching_3d
    cdff_dfn_bundle_adjustment
    cdff_dfn_voxelization
    cdff_dfn_disparity_to_pointcloud_with_intensity
    synthetic_generators
    cdff_dfpc_configurator
    cdff_dfpc_reconstruction_3d
    cdff_dfpc_point_cloud_model_localisation
    cdff_dfpc_haptic_scanning
)

if(PCL_FOUND)
    set(unittest_sources ${unittest_sources} "DFNs/ForceMeshGenerator/ForceMeshGenerator.cpp" "DFNs/ForceMeshGenerator/ForceMeshHelperFunctions.cpp")
    set(unittest_libraries ${unittest_libraries} cdff_dfn_force_mesh_generator pcl_io pcl_kdtree )
endif()


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
