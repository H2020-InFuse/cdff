/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file AdjustmentFromStereo.cpp
 * @date 15/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the AdjustmentFromStereo class.
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
#include "AdjustmentFromStereo.hpp"
#include <Errors/Assert.hpp>
#include <Errors/AssertOnTest.hpp>
#include <Types/CPP/VisualPointFeatureVector3D.hpp>

#include <Executors/ImageFiltering/ImageFilteringExecutor.hpp>
#include <Executors/StereoReconstruction/StereoReconstructionExecutor.hpp>
#include <Executors/FeaturesExtraction2D/FeaturesExtraction2DExecutor.hpp>
#include <Executors/FeaturesDescription2D/FeaturesDescription2DExecutor.hpp>
#include <Executors/FeaturesMatching2D/FeaturesMatching2DExecutor.hpp>
#include <Executors/BundleAdjustment/BundleAdjustmentExecutor.hpp>
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
using namespace FrameWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace MatrixWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
AdjustmentFromStereo::AdjustmentFromStereo() :
	EMPTY_FEATURE_VECTOR( NewVisualPointFeatureVector3D() ),
	LEFT_FEATURE_CATEGORY ( "orb_left" ),
	RIGHT_FEATURE_CATEGORY ( "orb_right" ),
	STEREO_CLOUD_CATEGORY( "stereo_cloud" ),
	TRIANGULATION_CLOUD_CATEGORY( "triangulation_cloud" ),
	cleanCorrespondenceMap( NewCorrespondenceMap2D() )
	{
	parameters = DEFAULT_PARAMETERS;

	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudMapResolution", parameters.pointCloudMapResolution, DEFAULT_PARAMETERS.pointCloudMapResolution);
	parametersHelper.AddParameter<float>("GeneralParameters", "SearchRadius", parameters.searchRadius, DEFAULT_PARAMETERS.searchRadius);
	parametersHelper.AddParameter<int>("GeneralParameters", "NumberOfAdjustedStereoPairs", parameters.numberOfAdjustedStereoPairs, DEFAULT_PARAMETERS.numberOfAdjustedStereoPairs);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseBundleInitialEstimation", parameters.useBundleInitialEstimation, DEFAULT_PARAMETERS.useBundleInitialEstimation);
	parametersHelper.AddParameter<float>("GeneralParameters", "Baseline", parameters.baseline, DEFAULT_PARAMETERS.baseline);

	currentInputNumber = 0;
	oldestCameraIndex = 0;
	firstTimeBundle = true;

	optionalLeftFilter = NULL;
	optionalRightFilter = NULL;
	reconstructor3d = NULL;
	featuresExtractor2d = NULL;
	optionalFeaturesDescriptor2d = NULL;
	featuresMatcher2d = NULL;
	bundleAdjuster = NULL;
	fundamentalMatrixComputer = NULL;
	perspectiveNPointSolver = NULL;
	reconstructor3dfrom2dmatches = NULL;

	bundleHistory = NULL;
	correspondencesRecorder = NULL;

	triangulatedKeypointCloud = NewPointCloud();

	estimatedCameraPoses = NULL;
	presentKeypointVector = NULL;
	keypointCloud = NULL;

	configurationFilePath = "";
	}

AdjustmentFromStereo::~AdjustmentFromStereo()
	{
	DeleteIfNotNull(bundleHistory);
	DeleteIfNotNull(correspondencesRecorder);
	DeleteIfNotNull(cleanCorrespondenceMap);
	DeleteIfNotNull(triangulatedKeypointCloud);

	DeleteIfNotNull(estimatedCameraPoses);
	DeleteIfNotNull(presentKeypointVector);
	DeleteIfNotNull(keypointCloud);

	delete(EMPTY_FEATURE_VECTOR);
	}


void AdjustmentFromStereo::run() 
	{
	DEBUG_PRINT_TO_LOG("Adjustment from stereo start", "");
 
	bundleHistory->AddImages(inLeftImage, inRightImage);

	FrameConstPtr filteredLeftImage = NULL;
	FrameConstPtr filteredRightImage = NULL;
	Executors::Execute(optionalLeftFilter, inLeftImage, filteredLeftImage);
	Executors::Execute(optionalRightFilter, inRightImage, filteredRightImage);

	ComputeStereoPointCloud(filteredLeftImage, filteredRightImage);
	ComputeVisualPointFeatures(filteredLeftImage, filteredRightImage);
	CreateWorkingCorrespondences();

	Poses3DSequenceConstPtr cameraPoses;
	if (currentInputNumber+1 < parameters.numberOfAdjustedStereoPairs)
		{
		outSuccess = false;
		}
	else
		{
		outSuccess = ComputeCameraPoses(cameraPoses);
		}

	if (!outSuccess && !firstTimeBundle)
		{
		bundleHistory->RemoveEntry(0);
		correspondencesRecorder->DiscardLatestCorrespondences();
		return;
		}

	if (outSuccess)
		{
		if(firstTimeBundle)
			{
			AddAllPointCloudsToMap(cameraPoses);
			firstTimeBundle = false;
			}
		else
			{
			AddLastPointCloudToMap(cameraPoses);
			}
		Copy( pointCloudMap.GetLatestPose(), outPose);
		PointCloudWrapper::PointCloudConstPtr outputPointCloud = pointCloudMap.GetScenePointCloudInOrigin(&outPose, parameters.searchRadius);
		Copy(*outputPointCloud, outPointCloud); 

		DEBUG_PRINT_TO_LOG("pose", ToString(outPose));
		DEBUG_PRINT_TO_LOG("points", GetNumberOfPoints(*outputPointCloud));

		DeleteIfNotNull(outputPointCloud);
		}

	currentInputNumber++;
	}

void AdjustmentFromStereo::setup()
	{
	configurator.configure(configurationFilePath);
	ConfigureExtraParameters(); //Configuration shall happen before alias assignment here.

	InstantiateDFNs();

	DeleteIfNotNull(bundleHistory);
	bundleHistory = new BundleHistory(parameters.numberOfAdjustedStereoPairs + 1);

	DeleteIfNotNull(correspondencesRecorder);
	correspondencesRecorder = new MultipleCorrespondences2DRecorder(parameters.numberOfAdjustedStereoPairs, true);	

	pointCloudMap.SetResolution(parameters.pointCloudMapResolution);

	SetPosition(rightToLeftCameraPose, -parameters.baseline, 0, 0);
	SetOrientation(rightToLeftCameraPose, 0, 0, 0, 1);

	if (parameters.useBundleInitialEstimation && estimatedCameraPoses == NULL)
		{
		estimatedCameraPoses = NewPoses3DSequence();
		presentKeypointVector = NewVisualPointFeatureVector2D();
		keypointCloud = NewPointCloud();
		}
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */

const AdjustmentFromStereo::AdjustmentFromStereoOptionsSet AdjustmentFromStereo::DEFAULT_PARAMETERS = 
	{
	/*.searchRadius =*/ 20,
	/*.pointCloudMapResolution =*/ 1e-2,
	/*.numberOfAdjustedStereoPairs =*/ 4,
	/*.useBundleInitialEstimation =*/ true,
	/*.baseline =*/ 1
	};

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void AdjustmentFromStereo::ConfigureExtraParameters()
	{
	parametersHelper.ReadFile( configurator.GetExtraParametersConfigurationFilePath() );

	ASSERT(parameters.pointCloudMapResolution > 0, "AdjustmentFromStereo Error, Point Cloud Map resolution is not positive");
	}

void AdjustmentFromStereo::InstantiateDFNs()
	{
	optionalLeftFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) );
	optionalRightFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) );
	reconstructor3d = static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3d") );
	featuresExtractor2d = static_cast<FeaturesExtraction2DInterface*>( configurator.GetDfn("featuresExtractor2d") );
	optionalFeaturesDescriptor2d = static_cast<FeaturesDescription2DInterface*>( configurator.GetDfn("featuresDescriptor2d", true) );
	featuresMatcher2d = static_cast<FeaturesMatching2DInterface*>( configurator.GetDfn("featuresMatcher2d") );
	bundleAdjuster = static_cast<BundleAdjustmentInterface*>( configurator.GetDfn("bundleAdjuster") );
	fundamentalMatrixComputer = static_cast<FundamentalMatrixComputationInterface*>( configurator.GetDfn("fundamentalMatrixComputer") );

	if (parameters.useBundleInitialEstimation)
		{		
		perspectiveNPointSolver = static_cast<PerspectiveNPointSolvingInterface*>( configurator.GetDfn("perspectiveNPointSolver") );
		reconstructor3dfrom2dmatches = static_cast<PointCloudReconstruction2DTo3DInterface*>( configurator.GetDfn("reconstructor3dfrom2dmatches") );
		}
	}

void AdjustmentFromStereo::ComputeStereoPointCloud(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage)
	{
	PointCloudConstPtr imageCloud = NULL;
	Executors::Execute(reconstructor3d, filteredLeftImage, filteredRightImage, imageCloud);

	bundleHistory->AddPointCloud(*imageCloud, STEREO_CLOUD_CATEGORY);

	DEBUG_PRINT_TO_LOG("Stereo points number", GetNumberOfPoints(*imageCloud));
	}

void AdjustmentFromStereo::ComputeVisualPointFeatures(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage)
	{
	VisualPointFeatureVector2DConstPtr keypointVector = NULL;
	VisualPointFeatureVector2DConstPtr featureVector = NULL;
	Executors::Execute(featuresExtractor2d, filteredLeftImage, keypointVector);
	Executors::Execute(optionalFeaturesDescriptor2d, filteredLeftImage, keypointVector, featureVector);
	bundleHistory->AddFeatures(*featureVector, LEFT_FEATURE_CATEGORY);
	DEBUG_PRINT_TO_LOG("Features Number", GetNumberOfPoints(*featureVector) );

	keypointVector = NULL;
	featureVector = NULL;
	Executors::Execute(featuresExtractor2d, filteredRightImage, keypointVector);
	Executors::Execute(optionalFeaturesDescriptor2d, filteredRightImage, keypointVector, featureVector);
	bundleHistory->AddFeatures(*featureVector, RIGHT_FEATURE_CATEGORY);
	DEBUG_PRINT_TO_LOG("Features Number", GetNumberOfPoints(*featureVector) );

	VisualPointFeatureVector2DConstPtr leftFeatureVector = bundleHistory->GetFeatures(0, LEFT_FEATURE_CATEGORY);
	VisualPointFeatureVector2DConstPtr rightFeatureVector = bundleHistory->GetFeatures(0, RIGHT_FEATURE_CATEGORY);
	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = NULL;
	Executors::Execute(featuresMatcher2d, leftFeatureVector, rightFeatureVector,leftRightCorrespondenceMap);
	DEBUG_PRINT_TO_LOG("Correspondences Number", GetNumberOfCorrespondences(*leftRightCorrespondenceMap) );

	MatrixWrapper::Matrix3dConstPtr fundamentalMatrix = NULL;
	bool success = false;
	CorrespondenceMap2DConstPtr inlierCorrespondenceMap = NULL;
	Executors::Execute(fundamentalMatrixComputer, leftRightCorrespondenceMap, fundamentalMatrix, success, inlierCorrespondenceMap);
	DEBUG_PRINT_TO_LOG("Inlier Correspondences Number", GetNumberOfCorrespondences(*inlierCorrespondenceMap) );

	if (parameters.useBundleInitialEstimation)
		{
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
		DEBUG_PRINT_TO_LOG("Triangulated points Number", GetNumberOfPoints(*triangulatedKeypointCloud) );
		bundleHistory->AddPointCloud(*triangulatedKeypointCloud, TRIANGULATION_CLOUD_CATEGORY);
		bundleHistory->AddMatches(*cleanCorrespondenceMap);
		}
	else
		{
		bundleHistory->AddMatches(*leftRightCorrespondenceMap);
		}
	}

void AdjustmentFromStereo::CleanUnmatchedFeatures(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr map, PointCloudWrapper::PointCloudPtr cloud)
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

void AdjustmentFromStereo::CreateWorkingCorrespondences()
	{
	VisualPointFeatureVector2DConstPtr leftFeatureVector = bundleHistory->GetFeatures(0, LEFT_FEATURE_CATEGORY);
	VisualPointFeatureVector2DConstPtr rightFeatureVector = bundleHistory->GetFeatures(0, RIGHT_FEATURE_CATEGORY);
	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = bundleHistory->GetMatches(0);

	correspondencesRecorder->InitializeNewSequence();
	correspondencesRecorder->AddCorrespondences(leftRightCorrespondenceMap);
	CreateWorkingCorrespondences(leftFeatureVector);
	CreateWorkingCorrespondences(rightFeatureVector);
	correspondencesRecorder->CompleteNewSequence();
	}

void AdjustmentFromStereo::CreateWorkingCorrespondences(VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr featureVector)
	{
	for(int backwardSteps = 1; backwardSteps < parameters.numberOfAdjustedStereoPairs; backwardSteps++)
		{
		VisualPointFeatureVector2DConstPtr pastLeftFeatureVector = bundleHistory->GetFeatures(backwardSteps, LEFT_FEATURE_CATEGORY);
		VisualPointFeatureVector2DConstPtr pastRightFeatureVector = bundleHistory->GetFeatures(backwardSteps, RIGHT_FEATURE_CATEGORY);

		if (pastLeftFeatureVector == NULL || pastRightFeatureVector == NULL)
			{
			DEBUG_PRINT_TO_LOG("Breaking at", backwardSteps);
			break;
			}

		CorrespondenceMap2DConstPtr leftTimeCorrespondenceMap = NULL;
		Executors::Execute(featuresMatcher2d, featureVector, pastLeftFeatureVector, leftTimeCorrespondenceMap);
		correspondencesRecorder->AddCorrespondences(leftTimeCorrespondenceMap);

		CorrespondenceMap2DConstPtr rightTimeCorrespondenceMap = NULL;
		Executors::Execute(featuresMatcher2d, featureVector, pastRightFeatureVector, rightTimeCorrespondenceMap);
		correspondencesRecorder->AddCorrespondences(rightTimeCorrespondenceMap);
		} 
	}

bool AdjustmentFromStereo::ComputeCameraPoses(PoseWrapper::Poses3DSequenceConstPtr& cameraPoses)
	{
	DEBUG_PRINT_TO_LOG("About to execute bundle adjustment", "");
	DEBUG_PRINT_TO_LOG("currentInputNumber", currentInputNumber );
	cameraPoses = NULL;
	bool success;
	float error;

	CorrespondenceMaps2DSequenceConstPtr workingCorrespondenceMapSequence = correspondencesRecorder->GetLatestCorrespondences();
	if (parameters.useBundleInitialEstimation)
		{
		PointCloudConstPtr triangulatedKeypointCloud = bundleHistory->GetPointCloud(0, TRIANGULATION_CLOUD_CATEGORY);
		PRINT_TO_LOG("Triangulated points Number", GetNumberOfPoints(*triangulatedKeypointCloud) );
		
		EstimateCameraPoses(); 

		Executors::Execute(bundleAdjuster, workingCorrespondenceMapSequence, estimatedCameraPoses, triangulatedKeypointCloud, cameraPoses, success, error);
		}
	else
		{
		Executors::Execute(bundleAdjuster, workingCorrespondenceMapSequence, cameraPoses, success, error);
		}
	
	if (success)
		{
		DEBUG_PRINT_TO_LOG("Bundle adjustement success", "");
		for(int poseIndex = 0; poseIndex< GetNumberOfPoses(*cameraPoses); poseIndex++)
			{
			DEBUG_PRINT_TO_LOG("Pose", ToString( GetPose(*cameraPoses, poseIndex) ) );
			}
		}
	else
		{
		DEBUG_PRINT_TO_LOG("Bundle adjustement failure", "");
		}

	DEBUG_PRINT_TO_LOG("Error:", error);

	return success;
	}

void AdjustmentFromStereo::AddAllPointCloudsToMap(Poses3DSequenceConstPtr& cameraPoses)
	{
	for(int stereoIndex = 0; stereoIndex < parameters.numberOfAdjustedStereoPairs; stereoIndex++)
		{
		const Pose3D& pose = GetPose(*cameraPoses, 2*stereoIndex);
		pointCloudMap.AddPointCloud( bundleHistory->GetPointCloud(stereoIndex, STEREO_CLOUD_CATEGORY), EMPTY_FEATURE_VECTOR, &pose);
		}
	}

void AdjustmentFromStereo::AddLastPointCloudToMap(Poses3DSequenceConstPtr& cameraPoses)
	{
	const Pose3D& poseOfPastCameraInCurrentCamera = GetPose(*cameraPoses, 2);

	//Inverting the pose.
	Pose3D poseOfCurrentCameraInPastCamera;
	SetPosition(poseOfCurrentCameraInPastCamera, -GetXPosition(poseOfPastCameraInCurrentCamera), -GetYPosition(poseOfPastCameraInCurrentCamera), -GetZPosition(poseOfPastCameraInCurrentCamera) );

	float qx = GetXOrientation(poseOfPastCameraInCurrentCamera);
	float qy = GetYOrientation(poseOfPastCameraInCurrentCamera);
	float qz = GetZOrientation(poseOfPastCameraInCurrentCamera);
	float qw = GetWOrientation(poseOfPastCameraInCurrentCamera);
	float squaredNorm = qx*qx + qy*qy + qz*qz + qw*qw;
	SetOrientation(poseOfCurrentCameraInPastCamera, -qx/squaredNorm, -qy/squaredNorm, -qz/squaredNorm, qw/squaredNorm);

	pointCloudMap.AttachPointCloud(bundleHistory->GetPointCloud(0, STEREO_CLOUD_CATEGORY), EMPTY_FEATURE_VECTOR, &poseOfCurrentCameraInPastCamera);
	}

void AdjustmentFromStereo::EstimateCameraPoses()
	{
	Pose3D zeroPose;
	SetPosition(zeroPose, 0, 0, 0);
	SetOrientation(zeroPose, 0, 0, 0, 1);
	Clear(*estimatedCameraPoses);
	
	VisualPointFeatureVector2DConstPtr leftFeatureVector = bundleHistory->GetFeatures(0, LEFT_FEATURE_CATEGORY);
	for(int backwardSteps = 1; backwardSteps < parameters.numberOfAdjustedStereoPairs; backwardSteps++)
		{
		VisualPointFeatureVector2DConstPtr pastLeftFeatureVector = bundleHistory->GetFeatures(backwardSteps, LEFT_FEATURE_CATEGORY);
		CorrespondenceMap2DConstPtr pastCorrespondenceMap = bundleHistory->GetMatches(backwardSteps);
		PointCloudConstPtr pastTriangulatedCloud = bundleHistory->GetPointCloud(backwardSteps, TRIANGULATION_CLOUD_CATEGORY);
		if (pastLeftFeatureVector == NULL)
			{
			break;
			}
		CorrespondenceMap2DConstPtr leftCorrespondenceMap = NULL;
		Executors::Execute(featuresMatcher2d, leftFeatureVector, pastLeftFeatureVector, leftCorrespondenceMap);

		Matrix3dConstPtr fundamentalMatrix = NULL;
		Pose3DConstPtr pose = NULL;
		bool success;
		CorrespondenceMap2DConstPtr inlierCorrespondenceMap = NULL;
		Executors::Execute(fundamentalMatrixComputer, leftCorrespondenceMap, fundamentalMatrix, success, inlierCorrespondenceMap);
		DEBUG_PRINT_TO_LOG("Number of inlier correspondences", GetNumberOfCorrespondences(*inlierCorrespondenceMap) );
		if (success)
			{
			EstimatePose(pastCorrespondenceMap, inlierCorrespondenceMap, pastTriangulatedCloud, pose, success);
			}
		if (success)
			{
			DEBUG_PRINT_TO_LOG("estimated pose", ToString(*pose));
			AddPose(*estimatedCameraPoses, *pose);
			}
		else
			{
			DEBUG_PRINT_TO_LOG("zero pose", ToString(zeroPose));
			AddPose(*estimatedCameraPoses, zeroPose);
			}
		}
	}

void AdjustmentFromStereo::EstimatePose(CorrespondenceMap2DConstPtr pastLeftRightCorrespondenceMap, CorrespondenceMap2DConstPtr leftPresentPastCorrespondenceMap, 
	PointCloudConstPtr pastCloud, Pose3DConstPtr& pose, bool& success)
	{
	int leftPresentPastCorrespondenceNumber = GetNumberOfCorrespondences(*leftPresentPastCorrespondenceMap);
	int pastLeftRightCorrespondenceNumber = GetNumberOfCorrespondences(*pastLeftRightCorrespondenceMap);
	ASSERT( GetNumberOfPoints(*pastCloud) == pastLeftRightCorrespondenceNumber, "Error: Cloud and correspondences do not have the same cardinality");

	ClearPoints(*presentKeypointVector);
	ClearPoints(*keypointCloud);
	for(int leftPresentPastIndex=0; leftPresentPastIndex<leftPresentPastCorrespondenceNumber; leftPresentPastIndex++)
		{
		BaseTypesWrapper::Point2D leftPoint = GetSource(*leftPresentPastCorrespondenceMap, leftPresentPastIndex);
		BaseTypesWrapper::Point2D pastLeftPoint = GetSink(*leftPresentPastCorrespondenceMap, leftPresentPastIndex);
		for(int pastLeftRightIndex=0; pastLeftRightIndex<pastLeftRightCorrespondenceNumber; pastLeftRightIndex++)
			{
			BaseTypesWrapper::Point2D correspondingPastLeftPoint = GetSource(*pastLeftRightCorrespondenceMap, pastLeftRightIndex);
			if ( correspondingPastLeftPoint.x == pastLeftPoint.x && correspondingPastLeftPoint.y == pastLeftPoint.y )
				{
				AddPoint(*presentKeypointVector, leftPoint.x, leftPoint.y);
				AddPoint(*keypointCloud, GetXCoordinate(*pastCloud, pastLeftRightIndex), GetYCoordinate(*pastCloud, pastLeftRightIndex), GetZCoordinate(*pastCloud, pastLeftRightIndex) );
				break;
				}
			}
		}

	pose = NULL;
	Executors::Execute(perspectiveNPointSolver, keypointCloud, presentKeypointVector, pose, success);
	}

}
}
}



/** @} */
