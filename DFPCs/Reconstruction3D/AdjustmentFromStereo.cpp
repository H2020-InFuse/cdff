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
#include "Errors/Assert.hpp"
#include <Visualizers/OpencvVisualizer.hpp>
#include <Visualizers/PclVisualizer.hpp>
#include <VisualPointFeatureVector3D.hpp>

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
	TRIANGULATION_CLOUD_CATEGORY( "triangulation_cloud" )
	{
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
	cameraTransformEstimator = NULL;
	reconstructor3dfrom2dmatches = NULL;

	bundleHistory = NULL;
	correspondencesRecorder = NULL;

	cleanCorrespondenceMap = NewCorrespondenceMap2D();
	triangulatedKeypointCloud = NewPointCloud();
	estimatedCameraPoses = NULL;

	configurationFilePath = "";
	}

AdjustmentFromStereo::~AdjustmentFromStereo()
	{
	delete(optionalLeftFilter);
	delete(optionalRightFilter);
	delete(reconstructor3d);
	delete(featuresExtractor2d);
	delete(optionalFeaturesDescriptor2d);
	delete(featuresMatcher2d);
	delete(bundleAdjuster);
	delete(fundamentalMatrixComputer);
	delete(cameraTransformEstimator);
	delete(reconstructor3dfrom2dmatches);

	DeleteIfNotNull(bundleHistory);
	DeleteIfNotNull(correspondencesRecorder);
	DeleteIfNotNull(cleanCorrespondenceMap);
	DeleteIfNotNull(triangulatedKeypointCloud);
	DeleteIfNotNull(estimatedCameraPoses);
	}


void AdjustmentFromStereo::run() 
	{
	#ifdef TESTING
	logFile.open("/InFuse/myLog.txt", std::ios::app);
	#endif
	DEBUG_PRINT_TO_LOG("Adjustment from stereo start", "");
 
	bundleHistory->AddImages(inLeftImage, inRightImage);

	FrameConstPtr filteredLeftImage = NULL;
	FrameConstPtr filteredRightImage = NULL;
	optionalLeftFilter->Execute(inLeftImage, filteredLeftImage);
	optionalRightFilter->Execute(inRightImage, filteredRightImage);

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
		#ifdef TESTING
		logFile << std::endl;
		logFile.close();
		#endif
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

		DEBUG_SHOW_POINT_CLOUD(outputPointCloud);
		DeleteIfNotNull(outputPointCloud);
		}

	currentInputNumber++;

	#ifdef TESTING
	logFile << std::endl;
	logFile.close();
	#endif
	}

void AdjustmentFromStereo::setup()
	{
	configurator.configure(configurationFilePath);
	ConfigureExtraParameters(); //Configuration shall happen before alias assignment here.

	InstantiateDFNExecutors();

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
	.searchRadius = 20,
	.pointCloudMapResolution = 1e-2,
	.numberOfAdjustedStereoPairs = 4,
	.useBundleInitialEstimation = true,
	.baseline = 1
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

void AdjustmentFromStereo::InstantiateDFNExecutors()
	{
	optionalLeftFilter = new ImageFilteringExecutor( static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) ) );
	optionalRightFilter = new ImageFilteringExecutor( static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) ) );
	reconstructor3d = new StereoReconstructionExecutor( static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3d") ) );
	featuresExtractor2d = new FeaturesExtraction2DExecutor( static_cast<FeaturesExtraction2DInterface*>( configurator.GetDfn("featuresExtractor2d") ) );
	optionalFeaturesDescriptor2d = new FeaturesDescription2DExecutor( static_cast<FeaturesDescription2DInterface*>( configurator.GetDfn("featuresDescriptor2d", true) ) );
	featuresMatcher2d = new FeaturesMatching2DExecutor( static_cast<FeaturesMatching2DInterface*>( configurator.GetDfn("featuresMatcher2d") ) );
	bundleAdjuster = new BundleAdjustmentExecutor( static_cast<BundleAdjustmentInterface*>( configurator.GetDfn("bundleAdjuster") ) );

	if (parameters.useBundleInitialEstimation)
		{
		fundamentalMatrixComputer = new FundamentalMatrixComputationExecutor( static_cast<FundamentalMatrixComputationInterface*>( configurator.GetDfn("fundamentalMatrixComputer") ) );
		cameraTransformEstimator = new CamerasTransformEstimationExecutor( static_cast<CamerasTransformEstimationInterface*>( configurator.GetDfn("cameraTransformEstimator") ) );
		reconstructor3dfrom2dmatches = new PointCloudReconstruction2DTo3DExecutor( static_cast<PointCloudReconstruction2DTo3DInterface*>( configurator.GetDfn("reconstructor3dfrom2dmatches") ) );
		}
	}

/**
* The method filters the left and right images, and uses them for the computation of a point cloud.
*
**/
void AdjustmentFromStereo::ComputeStereoPointCloud(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage)
	{
	PointCloudConstPtr imageCloud = NULL;
	reconstructor3d->Execute(filteredLeftImage, filteredRightImage, imageCloud);

	bundleHistory->AddPointCloud(*imageCloud, STEREO_CLOUD_CATEGORY);

	DEBUG_PRINT_TO_LOG("Stereo points number", GetNumberOfPoints(*imageCloud));
	//DEBUG_SHOW_POINT_CLOUD(imageCloud);
	}

#define MINIMUM(a, b) ( a < b ? a : b )

void AdjustmentFromStereo::ComputeVisualPointFeatures(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage)
	{
	VisualPointFeatureVector2DConstPtr keypointVector = NULL;
	VisualPointFeatureVector2DConstPtr featureVector = NULL;
	featuresExtractor2d->Execute(filteredLeftImage, keypointVector);
	optionalFeaturesDescriptor2d->Execute(filteredLeftImage, keypointVector, featureVector);
	bundleHistory->AddFeatures(*featureVector, LEFT_FEATURE_CATEGORY);
	PRINT_TO_LOG("Features Number", GetNumberOfPoints(*featureVector) );

	keypointVector = NULL;
	featureVector = NULL;
	featuresExtractor2d->Execute(filteredRightImage, keypointVector);
	optionalFeaturesDescriptor2d->Execute(filteredRightImage, keypointVector, featureVector);
	bundleHistory->AddFeatures(*featureVector, RIGHT_FEATURE_CATEGORY);
	PRINT_TO_LOG("Features Number", GetNumberOfPoints(*featureVector) );

	#ifdef TESTING
	logFile << "current" << " ";
	#endif

	VisualPointFeatureVector2DConstPtr leftFeatureVector = bundleHistory->GetFeatures(0, LEFT_FEATURE_CATEGORY);
	VisualPointFeatureVector2DConstPtr rightFeatureVector = bundleHistory->GetFeatures(0, RIGHT_FEATURE_CATEGORY);
	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = NULL;
	featuresMatcher2d->Execute(leftFeatureVector, rightFeatureVector,leftRightCorrespondenceMap);
	PRINT_TO_LOG("Correspondences Number", GetNumberOfCorrespondences(*leftRightCorrespondenceMap) );
	CleanLowScoringMatches(leftRightCorrespondenceMap, cleanCorrespondenceMap);
	PRINT_TO_LOG("Clean Correspondences Number", GetNumberOfCorrespondences(*cleanCorrespondenceMap) );

	if (parameters.useBundleInitialEstimation)
		{
		reconstructor3dfrom2dmatches->Execute(cleanCorrespondenceMap, &rightToLeftCameraPose, triangulatedKeypointCloud);
		//DEBUG_SHOW_2D_CORRESPONDENCES(filteredLeftImage, filteredRightImage, leftRightCorrespondenceMap);
		PRINT_TO_LOG("Triangulated points Number", GetNumberOfPoints(*triangulatedKeypointCloud) );
		CleanUnmatchedFeatures(cleanCorrespondenceMap, triangulatedKeypointCloud);
		bundleHistory->AddPointCloud(*triangulatedKeypointCloud, TRIANGULATION_CLOUD_CATEGORY);
		}
	
	bundleHistory->AddMatches(*cleanCorrespondenceMap);
	}

void AdjustmentFromStereo::CleanLowScoringMatches(CorrespondenceMap2DConstPtr leftRightCorrespondenceMap, CorrespondenceMap2DPtr cleanMap)
	{
	Copy(*leftRightCorrespondenceMap, *cleanMap);

	std::vector<BaseTypesWrapper::T_UInt32> removeIndexList;
	for(int correspondenceIndex1=0; correspondenceIndex1< GetNumberOfCorrespondences(*cleanMap); correspondenceIndex1++)
		{
		BaseTypesWrapper::Point2D source1 = GetSource(*cleanMap, correspondenceIndex1);
		BaseTypesWrapper::Point2D sink1 = GetSink(*cleanMap, correspondenceIndex1);
		if (source1.x != source1.x || source1.y != source1.y || sink1.x != sink1.x || sink1.y != sink1.y)
			{
			removeIndexList.push_back(correspondenceIndex1);
			continue;
			}
		bool found = false;
		for (int correspondenceIndex2=0; correspondenceIndex2<correspondenceIndex1 && !found; correspondenceIndex2++)
			{
			BaseTypesWrapper::Point2D source2 = GetSource(*cleanMap, correspondenceIndex2);
			BaseTypesWrapper::Point2D sink2 = GetSink(*cleanMap, correspondenceIndex2);
			if ( (source1.x == source2.x && source1.y == source2.y) || (sink1.x == sink2.x && sink1.y == sink2.y) )
				{
				removeIndexList.push_back(correspondenceIndex1);
				found = true;
				}
			}
		}
	RemoveCorrespondences(*cleanMap, removeIndexList);
	}

void AdjustmentFromStereo::CleanUnmatchedFeatures(CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr map, PointCloudWrapper::PointCloudPtr cloud)
	{
	ASSERT( GetNumberOfCorrespondences(*map) == GetNumberOfPoints(*cloud), "CleanUmatchedFeatures error: expected same number of points in map and cloud");
	
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
	RemoveCorrespondences(*map, removeIndexList);
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

	/*#ifdef TESTING
	std::vector<CorrespondenceMap2DConstPtr> correspondenceMapList = {leftRightCorrespondenceMap, leftTimeCorrespondenceMap, rightTimeCorrespondenceMap, pastLeftRightCorrespondenceMap};
	std::vector<FrameConstPtr> imageList = { 
		bundleHistory->GetLeftImage(0), bundleHistory->GetRightImage(0), bundleHistory->GetLeftImage(backwardSteps), bundleHistory->GetRightImage(backwardSteps) 
		};
	DEBUG_SHOW_QUADRUPLE_2D_CORRESPONDENCES( imageList, correspondenceMapList );
	#endif*/
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
		featuresMatcher2d->Execute(featureVector, pastLeftFeatureVector, leftTimeCorrespondenceMap);
		CleanLowScoringMatches(leftTimeCorrespondenceMap, cleanCorrespondenceMap);
		correspondencesRecorder->AddCorrespondences(cleanCorrespondenceMap);

		CorrespondenceMap2DConstPtr rightTimeCorrespondenceMap = NULL;
		featuresMatcher2d->Execute(featureVector, pastRightFeatureVector, rightTimeCorrespondenceMap);
		CleanLowScoringMatches(rightTimeCorrespondenceMap, cleanCorrespondenceMap);
		correspondencesRecorder->AddCorrespondences(cleanCorrespondenceMap);
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
	for(int mapIndex = 0; mapIndex < GetNumberOfCorrespondenceMaps(*workingCorrespondenceMapSequence); mapIndex++)
		{
		DEBUG_PRINT_TO_LOG(std::to_string(mapIndex), std::to_string( GetNumberOfCorrespondences( GetCorrespondenceMap(*workingCorrespondenceMapSequence, mapIndex) ) ) );
		}
	if (parameters.useBundleInitialEstimation)
		{
		PointCloudConstPtr triangulatedKeypointCloud = bundleHistory->GetPointCloud(0, TRIANGULATION_CLOUD_CATEGORY);
		PRINT_TO_LOG("Triangulated points Number", GetNumberOfPoints(*triangulatedKeypointCloud) );
		
		EstimateCameraPoses(); 

		bundleAdjuster->Execute(workingCorrespondenceMapSequence, estimatedCameraPoses, triangulatedKeypointCloud, cameraPoses, success, error);
		}
	else
		{
		bundleAdjuster->Execute(workingCorrespondenceMapSequence, cameraPoses, success, error);
		}

	#ifdef TESTING
	logFile << "corr" << " ";
	logFile << GetNumberOfCorrespondenceMaps(*workingCorrespondenceMapSequence) << " ";
	for(int i=0; i<GetNumberOfCorrespondenceMaps(*workingCorrespondenceMapSequence); i++)
		{
		const CorrespondenceMap2D& map = GetCorrespondenceMap(*workingCorrespondenceMapSequence, i);
		logFile << GetNumberOfCorrespondences(map) << " ";
		}
	#endif
	
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

	#ifdef TESTING
	logFile << error << " " << success << " ";
	#endif

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

	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = bundleHistory->GetMatches(0);
	for(int backwardSteps = 1; backwardSteps < parameters.numberOfAdjustedStereoPairs; backwardSteps++)
		{
		CorrespondenceMap2DConstPtr pastLeftRightCorrespondenceMap = bundleHistory->GetMatches(backwardSteps);
		if (pastLeftRightCorrespondenceMap == NULL)
			{
			break;
			}
		Matrix3dConstPtr fundamentalMatrix = NULL;
		Pose3DConstPtr pose = NULL;
		bool success;
		CorrespondenceMap2DConstPtr inlierCorrespondenceMap = NULL;
		fundamentalMatrixComputer->Execute(pastLeftRightCorrespondenceMap, fundamentalMatrix, success, inlierCorrespondenceMap);
		DEBUG_PRINT_TO_LOG("Number of inlier correspondences", GetNumberOfCorrespondences(*inlierCorrespondenceMap) );
		if (success)
			{
			cameraTransformEstimator->Execute(fundamentalMatrix, inlierCorrespondenceMap, pose, success);
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

}
}
}



/** @} */
