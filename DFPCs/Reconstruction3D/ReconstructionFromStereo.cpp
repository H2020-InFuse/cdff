/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ReconstructionFromStereo.cpp
 * @date 12/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the ReconstructionFromStereo class.
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "ReconstructionFromStereo.hpp"
#include "Errors/Assert.hpp"
#include <Visualizers/OpencvVisualizer.hpp>
#include <Visualizers/PclVisualizer.hpp>

#include <Executors/ImageFiltering/ImageFilteringExecutor.hpp>
#include <Executors/StereoReconstruction/StereoReconstructionExecutor.hpp>
#include <Executors/FeaturesMatching2D/FeaturesMatching2DExecutor.hpp>
#include <Executors/FeaturesExtraction2D/FeaturesExtraction2DExecutor.hpp>
#include <Executors/FeaturesDescription2D/FeaturesDescription2DExecutor.hpp>
#include <Executors/FundamentalMatrixComputation/FundamentalMatrixComputationExecutor.hpp>
#include <Executors/PerspectiveNPointSolving/PerspectiveNPointSolvingExecutor.hpp>
#include <Executors/PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DExecutor.hpp>

namespace CDFF
{
namespace DFPC
{
namespace Reconstruction3D
{

using namespace CDFF::DFN;
using namespace VisualPointFeatureVector2DWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace FrameWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace MatrixWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ReconstructionFromStereo::ReconstructionFromStereo() :
	EMPTY_FEATURE_VECTOR( NewVisualPointFeatureVector3D() ),
	LEFT_FEATURE_CATEGORY ( "orb_left" ),
	RIGHT_FEATURE_CATEGORY ( "orb_right" ),
	STEREO_CLOUD_CATEGORY( "stereo_cloud" ),
	TRIANGULATION_CLOUD_CATEGORY( "triangulation_cloud" )
	{
	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudMapResolution", parameters.pointCloudMapResolution, DEFAULT_PARAMETERS.pointCloudMapResolution);
	parametersHelper.AddParameter<float>("GeneralParameters", "SearchRadius", parameters.searchRadius, DEFAULT_PARAMETERS.searchRadius);
	parametersHelper.AddParameter<float>("GeneralParameters", "Baseline", parameters.baseline, DEFAULT_PARAMETERS.baseline);

	optionalLeftFilter = NULL;
	optionalRightFilter = NULL;
	featuresExtractor = NULL;
	featuresMatcher = NULL;
	fundamentalMatrixComputer = NULL;
	perspectiveNPointSolver = NULL;
	reconstructor3d = NULL;
	optionalFeaturesDescriptor = NULL;
	reconstructor3dfrom2dmatches = NULL;

	perspectiveCloud = NewPointCloud();
	perspectiveVector = NewVisualPointFeatureVector2D();
	triangulatedKeypointCloud = NewPointCloud();
	cleanCorrespondenceMap = NewCorrespondenceMap2D();

	bundleHistory = new BundleHistory(2);

	configurationFilePath = "";
	firstInput = true;
	#ifdef TESTING
	logFile.open("/InFuse/myLog.txt");
	logFile.close();
	#endif
	}

ReconstructionFromStereo::~ReconstructionFromStereo()
	{
	delete(perspectiveCloud);
	delete(perspectiveVector);
	delete(triangulatedKeypointCloud);
	delete(cleanCorrespondenceMap);

	delete(bundleHistory);
	
	delete(EMPTY_FEATURE_VECTOR);
	}

void ReconstructionFromStereo::run() 
	{
	#ifdef TESTING
	logFile.open("/InFuse/myLog.txt", std::ios::app);
	#endif
	DEBUG_PRINT_TO_LOG("Structure from stereo start", "");

	bundleHistory->AddImages(inLeftImage, inRightImage);

	FrameConstPtr filteredLeftImage = NULL;
	FrameConstPtr filteredRightImage = NULL;
	Executors::Execute(optionalLeftFilter, inLeftImage, filteredLeftImage);
	Executors::Execute(optionalRightFilter, inRightImage, filteredRightImage);

	PointCloudConstPtr imageCloud = NULL;
	Executors::Execute(reconstructor3d, filteredLeftImage, filteredRightImage, imageCloud);

	ComputeCurrentMatches(filteredLeftImage, filteredRightImage);

	if (firstInput)
		{
		firstInput = false;
		outSuccess = true;

		Pose3D zeroPose;
		SetPosition(zeroPose, 0, 0, 0);
		SetOrientation(zeroPose, 0, 0, 0, 1);
		pointCloudMap.AddPointCloud( imageCloud, EMPTY_FEATURE_VECTOR, &zeroPose);
		}
	else
		{
		Pose3DConstPtr previousPoseToPose = NULL;
		outSuccess = ComputeCameraMovement(previousPoseToPose);
		pointCloudMap.AttachPointCloud( imageCloud, EMPTY_FEATURE_VECTOR, previousPoseToPose);
		}

	if (outSuccess)
		{
		Copy( pointCloudMap.GetLatestPose(), outPose);
		PointCloudWrapper::PointCloudConstPtr outputPointCloud = pointCloudMap.GetScenePointCloudInOrigin(&outPose, parameters.searchRadius);
		Copy(*outputPointCloud, outPointCloud); 

		DEBUG_PRINT_TO_LOG("pose", ToString(outPose));
		DEBUG_PRINT_TO_LOG("points", GetNumberOfPoints(*outputPointCloud));

		DEBUG_SHOW_POINT_CLOUD(outputPointCloud);
		DeleteIfNotNull(outputPointCloud);
		}

	#ifdef TESTING
	logFile << std::endl;
	logFile.close();
	#endif
	}

void ReconstructionFromStereo::setup()
	{
	configurator.configure(configurationFilePath);
	InstantiateDFNs();
	ConfigureExtraParameters();

	pointCloudMap.SetResolution(parameters.pointCloudMapResolution);

	SetPosition(rightToLeftCameraPose, -parameters.baseline, 0, 0);
	SetOrientation(rightToLeftCameraPose, 0, 0, 0, 1);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */

const ReconstructionFromStereo::ReconstructionFromStereoOptionsSet ReconstructionFromStereo::DEFAULT_PARAMETERS = 
	{
	/*.searchRadius =*/ -1,
	/*.pointCloudMapResolution =*/ 1e-2,
	/*.baseline =*/ 1
	};

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void ReconstructionFromStereo::ConfigureExtraParameters()
	{
	parametersHelper.ReadFile( configurator.GetExtraParametersConfigurationFilePath() );

	ASSERT(parameters.pointCloudMapResolution > 0, "RegistrationFromStereo Error, Point Cloud Map resolution is not positive");
	}

void ReconstructionFromStereo::InstantiateDFNs()
	{
	optionalLeftFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) );
	optionalRightFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) );
	featuresExtractor = static_cast<FeaturesExtraction2DInterface*>( configurator.GetDfn("featuresExtractor") );
	featuresMatcher = static_cast<FeaturesMatching2DInterface*>( configurator.GetDfn("featuresMatcher") );
	fundamentalMatrixComputer = static_cast<FundamentalMatrixComputationInterface*>( configurator.GetDfn("fundamentalMatrixComputer") );
	perspectiveNPointSolver = static_cast<PerspectiveNPointSolvingInterface*>( configurator.GetDfn("perspectiveNPointSolver") );
	reconstructor3d = static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3D") );
	optionalFeaturesDescriptor = static_cast<FeaturesDescription2DInterface*>( configurator.GetDfn("featuresDescriptor", true) );
	reconstructor3dfrom2dmatches = static_cast<PointCloudReconstruction2DTo3DInterface*>( configurator.GetDfn("reconstructor3dfrom2dmatches") );
	}

void ReconstructionFromStereo::ComputeCurrentMatches(FrameConstPtr filteredLeftImage, FrameConstPtr filteredRightImage)
	{
	VisualPointFeatureVector2DConstPtr keypointVector = NULL;
	VisualPointFeatureVector2DConstPtr featureVector = NULL;
	Executors::Execute(featuresExtractor, filteredLeftImage, keypointVector);
	Executors::Execute(optionalFeaturesDescriptor, filteredLeftImage, keypointVector, featureVector);
	bundleHistory->AddFeatures(*featureVector, LEFT_FEATURE_CATEGORY);
	PRINT_TO_LOG("Features Number", GetNumberOfPoints(*featureVector) );

	#ifdef TESTING
	logFile << GetNumberOfPoints(*keypointVector) << " " << GetNumberOfPoints(*featureVector) << " ";
	#endif

	keypointVector = NULL;
	featureVector = NULL;
	Executors::Execute(featuresExtractor, filteredRightImage, keypointVector);
	Executors::Execute(optionalFeaturesDescriptor, filteredRightImage, keypointVector, featureVector);
	bundleHistory->AddFeatures(*featureVector, RIGHT_FEATURE_CATEGORY);
	DEBUG_PRINT_TO_LOG("Features Number", GetNumberOfPoints(*featureVector) );

	#ifdef TESTING
	logFile << GetNumberOfPoints(*keypointVector) << " " << GetNumberOfPoints(*featureVector) << " ";
	#endif

	VisualPointFeatureVector2DConstPtr leftFeatureVector = bundleHistory->GetFeatures(0, LEFT_FEATURE_CATEGORY);
	VisualPointFeatureVector2DConstPtr rightFeatureVector = bundleHistory->GetFeatures(0, RIGHT_FEATURE_CATEGORY);
	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = NULL;
	Executors::Execute(featuresMatcher, leftFeatureVector, rightFeatureVector,leftRightCorrespondenceMap);
	DEBUG_PRINT_TO_LOG("Correspondences Number", GetNumberOfCorrespondences(*leftRightCorrespondenceMap) );

	#ifdef TESTING
	logFile << GetNumberOfCorrespondences(*leftRightCorrespondenceMap) << " ";
	#endif

	MatrixWrapper::Matrix3dConstPtr fundamentalMatrix = NULL;
	bool success = false;
	CorrespondenceMap2DConstPtr inlierCorrespondenceMap = NULL;
	Executors::Execute(fundamentalMatrixComputer, leftRightCorrespondenceMap, fundamentalMatrix, success, inlierCorrespondenceMap);
	DEBUG_PRINT_TO_LOG("Inlier Correspondences Number", GetNumberOfCorrespondences(*inlierCorrespondenceMap) );

	if (success)
		{
		Executors::Execute(reconstructor3dfrom2dmatches, inlierCorrespondenceMap, &rightToLeftCameraPose, triangulatedKeypointCloud);
		CleanUnmatchedFeatures(inlierCorrespondenceMap, triangulatedKeypointCloud);
		}
	else
		{
		Executors::Execute(reconstructor3dfrom2dmatches, leftRightCorrespondenceMap, &rightToLeftCameraPose, triangulatedKeypointCloud);
		CleanUnmatchedFeatures(leftRightCorrespondenceMap, triangulatedKeypointCloud);
		}
	//DEBUG_SHOW_2D_CORRESPONDENCES(filteredLeftImage, filteredRightImage, leftRightCorrespondenceMap);
	DEBUG_PRINT_TO_LOG("Triangulated points Number", GetNumberOfPoints(*triangulatedKeypointCloud) );
	DEBUG_PRINT_TO_LOG("Clean Matches Number", GetNumberOfCorrespondences(*cleanCorrespondenceMap) );
	bundleHistory->AddPointCloud(*triangulatedKeypointCloud, TRIANGULATION_CLOUD_CATEGORY);
	bundleHistory->AddMatches(*cleanCorrespondenceMap);
	#ifdef TESTING
	logFile << GetNumberOfPoints(*triangulatedKeypointCloud) << " " << GetNumberOfCorrespondences(*cleanCorrespondenceMap) << " ";
	#endif
	}

bool ReconstructionFromStereo::ComputeCameraMovement(Pose3DConstPtr& previousPoseToPose)
	{
	VisualPointFeatureVector2DConstPtr leftFeatureVector = bundleHistory->GetFeatures(0, LEFT_FEATURE_CATEGORY);
	VisualPointFeatureVector2DConstPtr pastLeftFeatureVector = bundleHistory->GetFeatures(1, LEFT_FEATURE_CATEGORY);
	CorrespondenceMap2DConstPtr pastLeftCorrespondenceMap = NULL;
	Executors::Execute(featuresMatcher, leftFeatureVector, pastLeftFeatureVector, pastLeftCorrespondenceMap);
	DEBUG_PRINT_TO_LOG("Correspondences Number", GetNumberOfCorrespondences(*pastLeftCorrespondenceMap) );

	#ifdef TESTING
	logFile << GetNumberOfCorrespondences(*pastLeftCorrespondenceMap) << " ";
	#endif

	MatrixWrapper::Matrix3dConstPtr pastLeftFundamentalMatrix = NULL;
	bool pastSuccess = false;
	CorrespondenceMap2DConstPtr pastInlierCorrespondenceMap = NULL;
	Executors::Execute(fundamentalMatrixComputer, pastLeftCorrespondenceMap, pastLeftFundamentalMatrix, pastSuccess, pastInlierCorrespondenceMap);
	DEBUG_PRINT_TO_LOG("Inlier Correspondences Number", GetNumberOfCorrespondences(*pastInlierCorrespondenceMap) );
	
	CorrespondenceMap2DConstPtr pastCorrespondenceMap = pastSuccess ? pastInlierCorrespondenceMap : pastLeftCorrespondenceMap;
	CorrespondenceMap2DConstPtr currentCorrespondenceMap = bundleHistory->GetMatches(0);
	PointCloudConstPtr currentPointCloud = bundleHistory->GetPointCloud(0, TRIANGULATION_CLOUD_CATEGORY);

	ClearPoints(*perspectiveCloud);
	ClearPoints(*perspectiveVector);
	int numberOfPoints = GetNumberOfPoints(*currentPointCloud);
	int numberOfPastCorrespondences = GetNumberOfCorrespondences(*pastCorrespondenceMap);
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		BaseTypesWrapper::Point2D currentSource = GetSource(*currentCorrespondenceMap, pointIndex);
		BaseTypesWrapper::Point2D currentSink = GetSink(*currentCorrespondenceMap, pointIndex);
		for(int correspondenceIndex = 0; correspondenceIndex < numberOfPastCorrespondences; correspondenceIndex++)
			{
			BaseTypesWrapper::Point2D currentMatchedSource = GetSource(*pastCorrespondenceMap, correspondenceIndex);
			BaseTypesWrapper::Point2D pastSink = GetSink(*pastCorrespondenceMap, correspondenceIndex);
			if (currentSource.x == currentMatchedSource.x && currentSource.y == currentMatchedSource.y)
				{
				AddPoint(*perspectiveCloud, GetXCoordinate(*currentPointCloud, pointIndex), GetYCoordinate(*currentPointCloud, pointIndex), GetZCoordinate(*currentPointCloud, pointIndex));
				AddPoint(*perspectiveVector, pastSink.x, pastSink.y);
				}
			}
		}

	previousPoseToPose = NULL;
	bool success;
	DEBUG_PRINT_TO_LOG("Perspective cloud", GetNumberOfPoints(*perspectiveCloud) );
	DEBUG_PRINT_TO_LOG("Perspective vector", GetNumberOfPoints(*perspectiveVector) );
	Executors::Execute(perspectiveNPointSolver, perspectiveCloud, perspectiveVector, previousPoseToPose, success);
	DEBUG_PRINT_TO_LOG("Estimated pose", ToString(*previousPoseToPose));
	DEBUG_PRINT_TO_LOG("success", (success ? "yes" : "no") );
	return success;
	}

void ReconstructionFromStereo::CleanUnmatchedFeatures(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr map, PointCloudWrapper::PointCloudPtr cloud)
	{
	ASSERT( GetNumberOfCorrespondences(*map) == GetNumberOfPoints(*cloud), "CleanUmatchedFeatures error: expected same number of points in map and cloud");
	Copy(*map, *cleanCorrespondenceMap);	

	std::vector<BaseTypesWrapper::T_UInt32> removeIndexList;	
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*cloud); pointIndex++)
		{
		float x = GetXCoordinate(*cloud, pointIndex);
		float y = GetYCoordinate(*cloud, pointIndex);
		float z = GetZCoordinate(*cloud, pointIndex);
		if ( x != x || y != y || z != z)
			{
			removeIndexList.push_back(pointIndex);
			}
		}
	RemoveCorrespondences(*cleanCorrespondenceMap, removeIndexList);
	RemovePoints(*cloud, removeIndexList);
	}

}
}
}


/** @} */
