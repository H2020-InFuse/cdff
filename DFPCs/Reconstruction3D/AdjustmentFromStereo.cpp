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

#define DELETE_PREVIOUS(object) \
	{ \
	if (object != NULL) \
		{ \
		delete(object); \
		object = NULL; \
		} \
	} \

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
AdjustmentFromStereo::AdjustmentFromStereo()
	{
	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudMapResolution", parameters.pointCloudMapResolution, DEFAULT_PARAMETERS.pointCloudMapResolution);
	parametersHelper.AddParameter<float>("GeneralParameters", "SearchRadius", parameters.searchRadius, DEFAULT_PARAMETERS.searchRadius);
	parametersHelper.AddParameter<int>("GeneralParameters", "NumberOfAdjustedStereoPairs", parameters.numberOfAdjustedStereoPairs, DEFAULT_PARAMETERS.numberOfAdjustedStereoPairs);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseBundleInitialEstimation", parameters.useBundleInitialEstimation, DEFAULT_PARAMETERS.useBundleInitialEstimation);
	parametersHelper.AddParameter<float>("GeneralParameters", "Baseline", parameters.baseline, DEFAULT_PARAMETERS.baseline);

	leftImage = NewFrame();
	rightImage = NewFrame();
	filteredLeftImage = NULL;
	filteredRightImage = NULL;
	imagesCloud = NULL;
	leftKeypointsVector = NULL;
	rightKeypointsVector = NULL;
	leftFeaturesVector = NULL;
	rightFeaturesVector = NULL;
	historyCorrespondenceMaps = NULL;
	workingCorrespondenceMaps = NULL;
	latestCameraPoses = NewPoses3DSequence();
	emptyFeaturesVector = NewVisualPointFeatureVector3D();
	previousCameraPose = NewPose3D();

	fundamentalMatrix = NewMatrix3d();
	cameraTransform = NewPose3D();
	estimatedPointCloud = NewPointCloud();
	estimatedCameraPoses = NewPoses3DSequence();

	optionalLeftFilter = NULL;
	optionalRightFilter = NULL;
	reconstructor3d = NULL;
	featuresExtractor2d = NULL;
	optionalFeaturesDescriptor2d = NULL;
	featuresMatcher2d = NULL;
	bundleAdjuster = NULL;

	configurationFilePath = "";
	currentInputNumber = 0;
	oldestCameraIndex = 0;
	firstTimeBundle = true;
	}

AdjustmentFromStereo::~AdjustmentFromStereo()
	{
	if (optionalLeftFilter != NULL)
		{
		DELETE_PREVIOUS(filteredLeftImage);
		}
	if (optionalRightFilter != NULL)
		{
		DELETE_PREVIOUS(filteredRightImage);
		}
	if (optionalFeaturesDescriptor2d != NULL)
		{
		DELETE_PREVIOUS(leftFeaturesVector);
		DELETE_PREVIOUS(rightFeaturesVector);
		}
	delete(leftImage);
	delete(rightImage);
	DELETE_PREVIOUS(imagesCloud);
	DELETE_PREVIOUS(leftKeypointsVector);
	DELETE_PREVIOUS(rightKeypointsVector);
	DELETE_PREVIOUS(historyCorrespondenceMaps);
	DELETE_PREVIOUS(workingCorrespondenceMaps);
	delete(latestCameraPoses);
	DELETE_PREVIOUS(emptyFeaturesVector);
	DELETE_PREVIOUS(previousCameraPose);

	delete(fundamentalMatrix);
	delete(cameraTransform);
	delete(estimatedPointCloud);
	delete(estimatedCameraPoses);

	for(int imageIndex = 0; imageIndex < featuresVectorsList.size(); imageIndex++)
		{
		DELETE_PREVIOUS(featuresVectorsList.at(imageIndex));
		}
	for(int imageIndex = 0; imageIndex < featuresVectorsList.size(); imageIndex++)
		{
		std::vector<CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr>& mapsList = currentCorrespondenceMapsList.at(imageIndex);
		for(int correspondenceIndex = 0; correspondenceIndex < mapsList.size(); correspondenceIndex++)
			{
			DELETE_PREVIOUS( mapsList.at(correspondenceIndex) );
			}
		DELETE_PREVIOUS(featuresVectorsList.at(imageIndex));
		}
	for(int stereoIndex = 0; stereoIndex < pointCloudsList.size(); stereoIndex++)
		{
		DELETE_PREVIOUS(pointCloudsList.at(stereoIndex));
		}
	}


void AdjustmentFromStereo::run() 
	{
	#ifdef TESTING
	logFile.open("/InFuse/myLog.txt", std::ios::app);
	#endif
	DEBUG_PRINT_TO_LOG("Adjustment from stereo start", "");
 
	Copy(inLeftImage, *leftImage);
	Copy(inRightImage, *rightImage);

	FilterImages();
	ComputeStereoPointCloud();
	ComputeVisualPointFeatures();

	if (currentInputNumber+1 < parameters.numberOfAdjustedStereoPairs)
		{
		outSuccess = false;
		}
	else
		{
		outSuccess = ComputeCameraPoses();
		}

	if (outSuccess)
		{
		UpdateHistory();
		Pose3DConstPtr outputPose;
		if(firstTimeBundle)
			{
			outputPose = AddAllPointCloudsToMap();
			firstTimeBundle = false;
			}
		else
			{
			outputPose = AddLastPointCloudToMap();
			}
		Copy(*outputPose, outPose);

		PointCloudWrapper::PointCloudConstPtr outputPointCloud = pointCloudMap.GetScenePointCloudInOrigin(outputPose, parameters.searchRadius);
		Copy(*outputPointCloud, outPointCloud); 
		DEBUG_SHOW_POINT_CLOUD(outputPointCloud);
		DELETE_PREVIOUS(outputPointCloud);
		}
	else if (firstTimeBundle)
		{
		UpdateHistory();
		}
	else	
		{
		ClearDiscardedData();
		}
	#ifdef TESTING
	logFile << std::endl;
	logFile.close();
	#endif
	}

void AdjustmentFromStereo::setup()
	{
	configurator.configure(configurationFilePath);
	ConfigureExtraParameters(); //Configuration shall happen before alias assignment here.
	AssignDfnsAlias();
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
	pointCloudMap.SetResolution(parameters.pointCloudMapResolution);
	
	for(int imageIndex = 0; imageIndex < 2 * parameters.numberOfAdjustedStereoPairs; imageIndex++)
		{
		featuresVectorsList.push_back(NULL);
		currentCorrespondenceMapsList.push_back( std::vector<CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr>() );
		}
	for(int stereoIndex = 0; stereoIndex < parameters.numberOfAdjustedStereoPairs; stereoIndex++)
		{
		pointCloudsList.push_back(NULL);
		}
	}

void AdjustmentFromStereo::AssignDfnsAlias()
	{
	optionalLeftFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) );
	optionalRightFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) );
	reconstructor3d = static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3d") );
	featuresExtractor2d = static_cast<FeaturesExtraction2DInterface*>( configurator.GetDfn("featuresExtractor2d") );
	optionalFeaturesDescriptor2d = static_cast<FeaturesDescription2DInterface*>( configurator.GetDfn("featuresDescriptor2d", true) );
	featuresMatcher2d = static_cast<FeaturesMatching2DInterface*>( configurator.GetDfn("featuresMatcher2d") );
	bundleAdjuster = static_cast<BundleAdjustmentInterface*>( configurator.GetDfn("bundleAdjuster") );

	ASSERT(reconstructor3d != NULL, "DFPC Adjustment from stereo error: reconstructor3D DFN configured incorrectly");
	ASSERT(featuresExtractor2d != NULL, "DFPC Adjustment from stereo error: featuresExtractor2d DFN configured incorrectly");
	ASSERT(featuresMatcher2d != NULL, "DFPC Adjustment from stereo error: featuresMatcher3d DFN configured incorrectly");
	ASSERT(bundleAdjuster != NULL, "DFPC Adjustment from stereo error: bundleAdjuster DFN configured incorrectly");

	fundamentalMatrixComputer = static_cast<FundamentalMatrixComputationInterface*>( configurator.GetDfn("fundamentalMatrixComputer", true) );
	cameraTransformEstimator = static_cast<CamerasTransformEstimationInterface*>( configurator.GetDfn("cameraTransformEstimator", true) );
	reconstructor3dfrom2dmatches = static_cast<PointCloudReconstruction2DTo3DInterface*>( configurator.GetDfn("reconstructor3dfrom2dmatches", true) );

	if (parameters.useBundleInitialEstimation)
		{
		ASSERT(fundamentalMatrixComputer != NULL, "DFPC Adjustment from stereo error: fundamentalMatrixComputer DFN configured incorrectly");
		ASSERT(cameraTransformEstimator != NULL, "DFPC Adjustment from stereo error: cameraTransformEstimator DFN configured incorrectly");
		ASSERT(reconstructor3dfrom2dmatches != NULL, "DFPC Adjustment from stereo error: reconstructor3dfrom2dmatches DFN configured incorrectly");		
		}

	if (optionalLeftFilter != NULL)
		{
		filteredLeftImage = NewFrame();
		}
	if (optionalRightFilter != NULL)
		{
		filteredRightImage = NewFrame();
		}
	}

/**
* The method filters the left and right images, and uses them for the computation of a point cloud.
*
**/
void AdjustmentFromStereo::FilterImages()
	{
	FilterImage(leftImage, optionalLeftFilter, filteredLeftImage);
	FilterImage(rightImage, optionalRightFilter, filteredRightImage);
	}

void AdjustmentFromStereo::FilterImage(FramePtr image, ImageFilteringInterface* filter, FramePtr& filteredImage)
	{
	if (filter != NULL)
		{
		filter->imageInput(*image);
		filter->process();
		Copy(optionalLeftFilter->imageOutput(), *filteredImage);
		DEBUG_PRINT_TO_LOG("Filtered Frame", "");
		DEBUG_SHOW_IMAGE(filteredImage);
		}
	else
		{
		filteredImage = image;
		}
	}

void AdjustmentFromStereo::ComputeStereoPointCloud()
	{
	reconstructor3d->leftInput(*filteredLeftImage);
	reconstructor3d->rightInput(*filteredRightImage);
	reconstructor3d->process();

	PointCloudPtr newPointCloud = NewPointCloud();
	Copy(reconstructor3d->pointcloudOutput(), *newPointCloud);
	imagesCloud = newPointCloud;

	//Adding the extracted point cloud to storage
	imagesCloud = newPointCloud;
	if (currentInputNumber < parameters.numberOfAdjustedStereoPairs)
		{
		pointCloudsList.at(currentInputNumber) = imagesCloud;
		}
	else
		{
		DELETE_PREVIOUS( pointCloudsList.at(oldestCameraIndex/2) );
		pointCloudsList.at(oldestCameraIndex/2) = imagesCloud;
		}

	DEBUG_PRINT_TO_LOG("Point Cloud", GetNumberOfPoints(*imagesCloud));
	DEBUG_SHOW_POINT_CLOUD(imagesCloud);
	}

#define MINIMUM(a, b) ( a < b ? a : b )

void AdjustmentFromStereo::ComputeVisualPointFeatures()
	{
	ExtractFeatures(filteredLeftImage, leftKeypointsVector);
	ExtractFeatures(filteredRightImage, rightKeypointsVector);
	DescribeFeatures(filteredLeftImage, leftKeypointsVector, leftFeaturesVector);
	DescribeFeatures(filteredRightImage, rightKeypointsVector, rightFeaturesVector);

	//Adding the extracted features to storage
	int currentLeftCameraIndex, currentRightCameraIndex, mostRecentPastCameraIndex, endIndex;
	if (currentInputNumber < parameters.numberOfAdjustedStereoPairs)
		{
		currentLeftCameraIndex = 2 * currentInputNumber + 1;
		currentRightCameraIndex = 2 * currentInputNumber;
		mostRecentPastCameraIndex = (currentInputNumber > 0) ? 2 * currentInputNumber - 1 : 2 * parameters.numberOfAdjustedStereoPairs - 1;
		endIndex = 2 * parameters.numberOfAdjustedStereoPairs - 1;
		}
	else
		{
 		currentLeftCameraIndex = oldestCameraIndex + 1;
		currentRightCameraIndex = oldestCameraIndex;
		mostRecentPastCameraIndex = (2 * parameters.numberOfAdjustedStereoPairs + currentRightCameraIndex - 1) % (parameters.numberOfAdjustedStereoPairs * 2);
		endIndex = currentLeftCameraIndex;
		}

	//Computing the correspondences between current and past features and adding them to storage
	CorrespondenceMaps2DSequencePtr newSequence = NewCorrespondenceMaps2DSequence();
	for(int imageIndex = currentRightCameraIndex; imageIndex != endIndex; imageIndex = (2 * parameters.numberOfAdjustedStereoPairs + imageIndex - 1) % (parameters.numberOfAdjustedStereoPairs * 2) )
		{
		DEBUG_PRINT_TO_LOG("Matching", currentLeftCameraIndex);
		DEBUG_PRINT_TO_LOG("With", imageIndex);		
		CorrespondenceMap2DConstPtr newCorrespondence;
		if (imageIndex != currentRightCameraIndex)
			{
			newCorrespondence = MatchFeatures( leftFeaturesVector, featuresVectorsList.at(imageIndex) );
			}
		else
			{
			newCorrespondence = MatchFeatures( leftFeaturesVector, rightFeaturesVector );			
			}
		AddCorrespondenceMap(*newSequence, *newCorrespondence);
		}
	for(int imageIndex = mostRecentPastCameraIndex; imageIndex != endIndex; imageIndex = (2 * parameters.numberOfAdjustedStereoPairs + imageIndex - 1) % (parameters.numberOfAdjustedStereoPairs * 2) )
		{
		DEBUG_PRINT_TO_LOG("Matching", currentRightCameraIndex);
		DEBUG_PRINT_TO_LOG("With", imageIndex);	
		CorrespondenceMap2DConstPtr newCorrespondence = MatchFeatures( rightFeaturesVector, featuresVectorsList.at(imageIndex) );
		AddCorrespondenceMap(*newSequence, *newCorrespondence);
		}

	//Adding the old correspondences to thec current ones. All correspondences are taken except those with the oldest left and right images
	if (0 < currentInputNumber && currentInputNumber < parameters.numberOfAdjustedStereoPairs)
		{
		for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondenceMaps(*historyCorrespondenceMaps); correspondenceIndex++)
			{
			AddCorrespondenceMap(*newSequence, GetCorrespondenceMap(*historyCorrespondenceMaps, correspondenceIndex));
			}
		}
	else if (currentInputNumber > 0)
		{
		int correspondenceIndex = 0;
		for(int sourceIndex = 0; sourceIndex < 2 * (parameters.numberOfAdjustedStereoPairs - 1); sourceIndex++)
			{
			for(int sinkIndex = sourceIndex + 1; sinkIndex < 2 * (parameters.numberOfAdjustedStereoPairs - 1); sinkIndex++)
				{
				AddCorrespondenceMap(*newSequence, GetCorrespondenceMap(*historyCorrespondenceMaps, correspondenceIndex));	
				correspondenceIndex++;
				}
			correspondenceIndex += 2;
			}
		}

	//Updating the historyCorrespondenceMaps
	DELETE_PREVIOUS(workingCorrespondenceMaps);
	workingCorrespondenceMaps = newSequence;
	DEBUG_PRINT_TO_LOG("number of working correspondence maps", GetNumberOfCorrespondenceMaps(*workingCorrespondenceMaps));
	}

void AdjustmentFromStereo::ExtractFeatures(FrameWrapper::FrameConstPtr filteredImage, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr& keypointsVector)
	{
	featuresExtractor2d->frameInput(*filteredImage);
	featuresExtractor2d->process();

	VisualPointFeatureVector2DPtr newKeypointsVector = NewVisualPointFeatureVector2D();
	Copy(featuresExtractor2d->featuresOutput(), *newKeypointsVector);
	keypointsVector = newKeypointsVector;

	DEBUG_PRINT_TO_LOG("Extracted Current Features", GetNumberOfPoints(*keypointsVector) );
	}

void AdjustmentFromStereo::DescribeFeatures(FrameConstPtr image, VisualPointFeatureVector2DConstPtr keypointsVector, VisualPointFeatureVector2DConstPtr& featuresVector)
	{
	if (optionalFeaturesDescriptor2d != NULL)
		{
		optionalFeaturesDescriptor2d->frameInput(*image);
		optionalFeaturesDescriptor2d->featuresInput(*keypointsVector);
		optionalFeaturesDescriptor2d->process();

		VisualPointFeatureVector2DPtr newFeaturesVector = NewVisualPointFeatureVector2D();
		Copy(optionalFeaturesDescriptor2d->featuresOutput(), *newFeaturesVector);
		featuresVector = newFeaturesVector;

		DEBUG_PRINT_TO_LOG("Described Current Features", GetNumberOfPoints(*featuresVector) );
		DELETE_PREVIOUS(keypointsVector); //Keypoints are no longer needed
		}
	else
		{
		featuresVector = keypointsVector;
		}
	}

CorrespondenceMap2DConstPtr AdjustmentFromStereo::MatchFeatures(VisualPointFeatureVector2DConstPtr sourceFeaturesVector, VisualPointFeatureVector2DConstPtr sinkFeaturesVector)
	{
	featuresMatcher2d->sourceFeaturesInput(*sourceFeaturesVector);
	featuresMatcher2d->sinkFeaturesInput(*sinkFeaturesVector);
	featuresMatcher2d->process();
	CorrespondenceMap2DPtr newCorrespondence = NewCorrespondenceMap2D();
	Copy(featuresMatcher2d->matchesOutput(), *newCorrespondence);	
	DEBUG_PRINT_TO_LOG("Correspondences", GetNumberOfCorrespondences(*newCorrespondence) );

	//Cleaning repeated source and sink points within the same map
	std::vector<BaseTypesWrapper::T_UInt32> removeIndexList;
	for(int correspondenceIndex1=0; correspondenceIndex1< GetNumberOfCorrespondences(*newCorrespondence); correspondenceIndex1++)
		{
		for (int correspondenceIndex2=0; correspondenceIndex2<correspondenceIndex1; correspondenceIndex2++)
			{
			BaseTypesWrapper::Point2D source1 = GetSource(*newCorrespondence, correspondenceIndex1);
			BaseTypesWrapper::Point2D sink1 = GetSink(*newCorrespondence, correspondenceIndex1);
			BaseTypesWrapper::Point2D source2 = GetSource(*newCorrespondence, correspondenceIndex2);
			BaseTypesWrapper::Point2D sink2 = GetSink(*newCorrespondence, correspondenceIndex2);
			if ( (source1.x == source2.x && source1.y == source2.y) || (sink1.x == sink2.x && sink1.y == sink2.y) )
				{
				if ( removeIndexList.size() == 0 || correspondenceIndex1 != removeIndexList.at( removeIndexList.size()-1 ) )
					{
					removeIndexList.push_back(correspondenceIndex1);
					}
				}
			}
		}
	RemoveCorrespondences(*newCorrespondence, removeIndexList);

	return newCorrespondence;
	}

bool AdjustmentFromStereo::ComputeCameraPoses()
	{
	DEBUG_PRINT_TO_LOG("About to execute bundle adjustment", "");
	DEBUG_PRINT_TO_LOG("currentInputNumber", currentInputNumber );
	if (parameters.useBundleInitialEstimation)
		{
		EstimatePointCloud(); //Estimates point cloud from the most recent image pair
		EstimateCameraPoses(); //Estimates the camera poses with respect to the most recent pose.
		CleanBundleAdjustmentInputs(); //Cleaning is needed because there may be correspondences without a valid 3d point.

		bundleAdjuster->guessedPosesSequenceInput(*estimatedCameraPoses);
		bundleAdjuster->guessedPointCloudInput(*estimatedPointCloud);
		}
	#ifdef TESTING
	logFile << "corr" << " ";
	logFile << GetNumberOfCorrespondenceMaps(*workingCorrespondenceMaps) << " ";
	for(int i=0; i<GetNumberOfCorrespondenceMaps(*workingCorrespondenceMaps); i++)
		{
		const CorrespondenceMap2D& map = GetCorrespondenceMap(*workingCorrespondenceMaps, i);
		logFile << GetNumberOfCorrespondences(map) << " ";
		}
	#endif

	bundleAdjuster->correspondenceMapsSequenceInput(*workingCorrespondenceMaps);
	bundleAdjuster->process();
	bool successOutput = bundleAdjuster->successOutput();
	
	if (successOutput)
		{
		Copy(bundleAdjuster->posesSequenceOutput(), *latestCameraPoses);
		DEBUG_PRINT_TO_LOG("Bundle adjustement success", "");
		}
	else
		{
		DEBUG_PRINT_TO_LOG("Bundle adjustement failure", "");
		}

	DEBUG_PRINT_TO_LOG("Error:", bundleAdjuster->errorOutput());

	#ifdef TESTING
	logFile << bundleAdjuster->errorOutput() << " " << successOutput << " ";
	#endif

	return successOutput;
	}

PoseWrapper::Pose3DConstPtr AdjustmentFromStereo::AddAllPointCloudsToMap()
	{
	for(int stereoIndex = 0; stereoIndex < parameters.numberOfAdjustedStereoPairs; stereoIndex++)
		{
		const Pose3D& pose = GetPose(*latestCameraPoses, parameters.numberOfAdjustedStereoPairs - 1 - stereoIndex);
		pointCloudMap.AddPointCloud(pointCloudsList.at(stereoIndex), emptyFeaturesVector, &pose);
		}
	Copy(GetPose(*latestCameraPoses, 0), *previousCameraPose);
	return previousCameraPose;
	}

PoseWrapper::Pose3DConstPtr AdjustmentFromStereo::AddLastPointCloudToMap()
	{
	const Pose3D& inversePose = GetPose(*latestCameraPoses, 0);
	
	Pose3D lastPose;
	SetPosition(lastPose, -GetXPosition(inversePose), -GetYPosition(inversePose), -GetZPosition(inversePose) );
	
	float qx = GetXOrientation(inversePose);
	float qy = GetYOrientation(inversePose);
	float qz = GetZOrientation(inversePose);
	float qw = GetWOrientation(inversePose);
	float squaredNorm = qx*qx + qy*qy + qz*qz + qw*qw;
	SetOrientation(lastPose, -qx/squaredNorm, -qy/squaredNorm, -qz/squaredNorm, qw/squaredNorm);

	pointCloudMap.AddPointCloud(imagesCloud, emptyFeaturesVector, &lastPose);
	return previousCameraPose;
	}

bool AdjustmentFromStereo::EstimatePointCloud()
	{
	const CorrespondenceMap2D& recentLeftRightCorrespondence = GetCorrespondenceMap( *workingCorrespondenceMaps, 0 );
	ComputeStereoPointCloud( &recentLeftRightCorrespondence );
	bool thereIsOne3dPointForEachCorrespondence = GetNumberOfPoints(*estimatedPointCloud) == GetNumberOfCorrespondences( recentLeftRightCorrespondence );
	VERIFY( thereIsOne3dPointForEachCorrespondence, 
		"AdjustmentFromStereo error, the reconstructor3dTo2d you used does not output a 3d point for each correspondence, please adjust the option setting or use another DFN");
	return thereIsOne3dPointForEachCorrespondence;
	}

bool AdjustmentFromStereo::EstimateCameraPoses()
	{
	int numberOfCameras = featuresVectorsList.size();
	int numberOfStereoCameras = numberOfCameras/2;
	int pastLeftCorrespondenceIndex = 0;
	Clear(*estimatedCameraPoses);
	for(int stereoIndex = 0; stereoIndex < numberOfStereoCameras - 1; stereoIndex++)
		{
		pastLeftCorrespondenceIndex += (numberOfCameras - 2*stereoIndex - 1) + (numberOfCameras - 2*stereoIndex - 2);
		const CorrespondenceMap2D& leftPastLeftCorrespondence = GetCorrespondenceMap( *workingCorrespondenceMaps, pastLeftCorrespondenceIndex );
		bool success = ComputeFundamentalMatrix(&leftPastLeftCorrespondence, fundamentalMatrix);
		success = success && ComputeCameraTransform(&leftPastLeftCorrespondence, fundamentalMatrix, cameraTransform);
		if (!success)
			{	
			return false;
			}		
		AddPose(*estimatedCameraPoses, *cameraTransform);
		}
	return true;
	}

bool AdjustmentFromStereo::ComputeFundamentalMatrix(CorrespondenceMap2DConstPtr inputCorrespondenceMap, Matrix3dPtr outputFundamentalMatrix)
	{
	if (GetNumberOfCorrespondences(*inputCorrespondenceMap) < 8 )
		{
		return false;
		}
	fundamentalMatrixComputer->matchesInput(*inputCorrespondenceMap);
	fundamentalMatrixComputer->process();
	Copy( fundamentalMatrixComputer->fundamentalMatrixOutput(), *outputFundamentalMatrix);	
	bool fundamentalMatrixSuccess =  fundamentalMatrixComputer->successOutput();
	DEBUG_PRINT_TO_LOG("Fundamental Matrix", fundamentalMatrixSuccess);
	if(fundamentalMatrixSuccess)
		{
		DEBUG_SHOW_MATRIX(outputFundamentalMatrix);
		}
	return fundamentalMatrixSuccess;
	}

bool AdjustmentFromStereo::ComputeCameraTransform(CorrespondenceMap2DConstPtr inputCorrespondenceMap, Matrix3dPtr inputFundamentalMatrix, Pose3DPtr outputCameraTransform)
	{
	cameraTransformEstimator->fundamentalMatrixInput(*inputFundamentalMatrix);
	cameraTransformEstimator->matchesInput(*inputCorrespondenceMap);
	cameraTransformEstimator->process();
	Copy( cameraTransformEstimator->transformOutput(), *outputCameraTransform);
	bool essentialMatrixSuccess = cameraTransformEstimator->successOutput();
	DEBUG_PRINT_TO_LOG("Essential Matrix", essentialMatrixSuccess);
	if(essentialMatrixSuccess)
		{
		DEBUG_SHOW_POSE(outputCameraTransform);
		}
	return essentialMatrixSuccess;
	}

void AdjustmentFromStereo::ComputeStereoPointCloud(CorrespondenceMap2DConstPtr inputCorrespondenceMap)
	{
	Pose3D rightCameraPose;
	SetPosition(rightCameraPose, -parameters.baseline, 0, 0);
	SetOrientation(rightCameraPose, 0, 0, 0, 1);

	reconstructor3dfrom2dmatches->poseInput(rightCameraPose);
	reconstructor3dfrom2dmatches->matchesInput(*inputCorrespondenceMap);
	reconstructor3dfrom2dmatches->process();
	Copy( reconstructor3dfrom2dmatches->pointcloudOutput(), *estimatedPointCloud);
	DEBUG_PRINT_TO_LOG("Left Right Point Cloud", GetNumberOfPoints(*estimatedPointCloud));
	DEBUG_SHOW_POINT_CLOUD(estimatedPointCloud);	
	}

void AdjustmentFromStereo::CleanBundleAdjustmentInputs()
	{
	// Removing invalid 3d points
	std::vector<BaseTypesWrapper::T_UInt32> indexToRemoveList;
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*estimatedPointCloud); pointIndex++)
		{
		if ( GetXCoordinate(*estimatedPointCloud, pointIndex) != GetXCoordinate(*estimatedPointCloud, pointIndex) )
			{
			indexToRemoveList.push_back(pointIndex);
			}
		}
	RemoveCorrespondences(*workingCorrespondenceMaps, 0, indexToRemoveList);
	RemovePoints(*estimatedPointCloud, indexToRemoveList);

	//Removing repeated source points
	for(int mapIndex = 0; mapIndex < GetNumberOfCorrespondenceMaps(*workingCorrespondenceMaps); mapIndex++)
		{
		indexToRemoveList.clear();
		const CorrespondenceMap2D& correspondenceMap = GetCorrespondenceMap(*workingCorrespondenceMaps, mapIndex);
		for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(correspondenceMap); correspondenceIndex++)
			{
			BaseTypesWrapper::Point2D sourcePoint = GetSource(correspondenceMap, correspondenceIndex);
			bool found = false;
			for(int secondCorrespondenceIndex = correspondenceIndex+1; secondCorrespondenceIndex < GetNumberOfCorrespondences(correspondenceMap) && !found; secondCorrespondenceIndex++)
				{
				BaseTypesWrapper::Point2D secondSourcePoint = GetSource(correspondenceMap, secondCorrespondenceIndex);
				if (StaticCastToInt(sourcePoint.x) == StaticCastToInt(secondSourcePoint.x) && StaticCastToInt(sourcePoint.y) == StaticCastToInt(secondSourcePoint.y))
					{
					indexToRemoveList.push_back(correspondenceIndex);
					found = true;
					}
				}
			}
		RemoveCorrespondences(*workingCorrespondenceMaps, mapIndex, indexToRemoveList);
		if (mapIndex == 0)
			{
			RemovePoints(*estimatedPointCloud, indexToRemoveList);
			} 
		}

	//Removing repeated sink points
	for(int mapIndex = 0; mapIndex < GetNumberOfCorrespondenceMaps(*workingCorrespondenceMaps); mapIndex++)
		{
		indexToRemoveList.clear();
		const CorrespondenceMap2D& correspondenceMap = GetCorrespondenceMap(*workingCorrespondenceMaps, mapIndex);
		for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(correspondenceMap); correspondenceIndex++)
			{
			BaseTypesWrapper::Point2D sinkPoint = GetSink(correspondenceMap, correspondenceIndex);
			bool found = false;
			for(int secondCorrespondenceIndex = correspondenceIndex+1; secondCorrespondenceIndex < GetNumberOfCorrespondences(correspondenceMap) && !found; secondCorrespondenceIndex++)
				{
				BaseTypesWrapper::Point2D secondSinkPoint = GetSink(correspondenceMap, secondCorrespondenceIndex);
				if (StaticCastToInt(sinkPoint.x) == StaticCastToInt(secondSinkPoint.x) && StaticCastToInt(sinkPoint.y) == StaticCastToInt(secondSinkPoint.y))
					{
					indexToRemoveList.push_back(correspondenceIndex);
					found = true;
					}
				}
			}
		RemoveCorrespondences(*workingCorrespondenceMaps, mapIndex, indexToRemoveList);
		if (mapIndex == 0)
			{
			RemovePoints(*estimatedPointCloud, indexToRemoveList);
			} 
		}

	DEBUG_PRINT_TO_LOG("Number of working correspondences", GetNumberOfCorrespondenceMaps(*workingCorrespondenceMaps) );
	DEBUG_PRINT_TO_LOG("Number of historical correspondences", GetNumberOfCorrespondenceMaps(*historyCorrespondenceMaps) );
	}

int AdjustmentFromStereo::StaticCastToInt(float value)
	{
	int integerValue = 0;
	if (value > 0)
		{
		while( value >= 0.5 )
			{
			integerValue++;
			value = value - 1;
			}
		return integerValue;
		}

	while (value <= -0.5)
		{
		integerValue--;
		value = value + 1;
		}
	return integerValue;
	}

void AdjustmentFromStereo::UpdateHistory()
	{
	PRINT_TO_LOG("Updating history on currentInputNumber", currentInputNumber);
	//Adding the extracted features to storage
	int currentLeftCameraIndex, currentRightCameraIndex;
	if (currentInputNumber < parameters.numberOfAdjustedStereoPairs)
		{
		currentLeftCameraIndex = 2 * currentInputNumber + 1;
		currentRightCameraIndex = 2 * currentInputNumber;
		oldestCameraIndex = 0;
		}
	else
		{
 		currentLeftCameraIndex = oldestCameraIndex + 1;
		currentRightCameraIndex = oldestCameraIndex;
		oldestCameraIndex = (oldestCameraIndex + 2) % (parameters.numberOfAdjustedStereoPairs * 2);
		}
	DELETE_PREVIOUS( featuresVectorsList.at(currentLeftCameraIndex) );
	DELETE_PREVIOUS( featuresVectorsList.at(currentRightCameraIndex) );
	featuresVectorsList.at(currentLeftCameraIndex) = leftFeaturesVector;
	featuresVectorsList.at(currentRightCameraIndex) = rightFeaturesVector;

	//Updating history correspondence map
	DELETE_PREVIOUS(historyCorrespondenceMaps);
	historyCorrespondenceMaps = workingCorrespondenceMaps;
	workingCorrespondenceMaps = NULL; //So that it won't be deleted

	//Updating currentInputNumber for next round
	//if (currentInputNumber < parameters.numberOfAdjustedStereoPairs)
		{
		currentInputNumber++;
		}
	}

void AdjustmentFromStereo::ClearDiscardedData()
	{
	DELETE_PREVIOUS( leftFeaturesVector );	
	DELETE_PREVIOUS( rightFeaturesVector );
	}

}
}
}



/** @} */
