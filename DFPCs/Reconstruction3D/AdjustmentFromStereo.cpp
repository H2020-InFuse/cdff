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

namespace dfpc_ci {

using namespace dfn_ci;
using namespace FrameWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace CorrespondenceMap2DWrapper;

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

	filteredLeftImage = NULL;
	filteredRightImage = NULL;
	imagesCloud = NULL;
	leftKeypointsVector = NULL;
	rightKeypointsVector = NULL;
	leftFeaturesVector = NULL;
	rightFeaturesVector = NULL;
	latestCorrespondenceMaps = NULL;
	latestCameraPoses = NULL;
	emptyFeaturesVector = NewVisualPointFeatureVector3D();
	previousCameraPose = NewPose3D();

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
	DELETE_PREVIOUS(imagesCloud);
	DELETE_PREVIOUS(leftKeypointsVector);
	DELETE_PREVIOUS(rightKeypointsVector);
	DELETE_PREVIOUS(latestCorrespondenceMaps);
	DELETE_PREVIOUS(latestCameraPoses);
	DELETE_PREVIOUS(emptyFeaturesVector);
	DELETE_PREVIOUS(previousCameraPose);
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
	DEBUG_PRINT_TO_LOG("Adjustment from stereo start", "");

	leftImage = inLeftImage;
	rightImage = inRightImage;

	FilterImages();
	ComputeStereoPointCloud();
	ComputeVisualPointFeatures();

	if (currentInputNumber < parameters.numberOfAdjustedStereoPairs)
		{
		currentInputNumber++;
		outSuccess = false;
		}
	else
		{
		outSuccess = ComputeCameraPoses();
		}

	if (outSuccess)
		{
		if(firstTimeBundle)
			{
			outPose = AddAllPointCloudsToMap();
			}
		else
			{
			outPose = AddLastPointCloudToMap();
			}

		outPointCloud = pointCloudMap.GetScenePointCloudInOrigin(outPose, parameters.searchRadius);
		DEBUG_SHOW_POINT_CLOUD(outPointCloud);
		}
	else
		{
		outPointCloud = NULL;
		outPose = NULL;
		}
	}

void AdjustmentFromStereo::setup()
	{
	configurator.configure(configurationFilePath);
	AssignDfnsAlias();
	ConfigureExtraParameters();
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
	.numberOfAdjustedStereoPairs = 4
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

void AdjustmentFromStereo::FilterImage(FrameConstPtr image, ImageFilteringInterface* filter, FrameConstPtr& filteredImage)
	{
	if (filter != NULL)
		{
		filter->imageInput(*image);
		filter->process();

		DELETE_PREVIOUS(filteredImage);
		FramePtr newFrame = NewFrame();
		Copy(optionalLeftFilter->imageOutput(), *newFrame);
		filteredImage = newFrame;

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
	reconstructor3d->leftImageInput(filteredLeftImage);
	reconstructor3d->rightImageInput(filteredRightImage);
	reconstructor3d->process();

	//PointCloudPtr newPointCloud = NewPointCloud();
	//Copy(reconstructor3d->pointCloudOutput(), *newPointCloud);
	//imagesCloud = newPointCloud;

	//Adding the extracted point cloud to storage
	imagesCloud = reconstructor3d->pointCloudOutput();
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
	int currentLeftCameraIndex, currentRightCameraIndex, mostRecentPastCameraIndex;
	if (currentInputNumber < parameters.numberOfAdjustedStereoPairs)
		{
		currentLeftCameraIndex = 2 * currentInputNumber + 1;
		currentRightCameraIndex = 2 * currentInputNumber;
		mostRecentPastCameraIndex = 2 * currentInputNumber - 1;
		oldestCameraIndex = 0;
		}
	else
		{
 		currentLeftCameraIndex = oldestCameraIndex + 1;
		currentRightCameraIndex = oldestCameraIndex;
		mostRecentPastCameraIndex = (currentRightCameraIndex - 1) % (parameters.numberOfAdjustedStereoPairs * 2);
		oldestCameraIndex = (oldestCameraIndex + 2) % (parameters.numberOfAdjustedStereoPairs * 2);
		}
	featuresVectorsList.at(currentLeftCameraIndex) = leftFeaturesVector;
	featuresVectorsList.at(currentRightCameraIndex) = rightFeaturesVector;

	//Computing the correspondences between current and past features and adding them to storage
	CorrespondenceMaps2DSequencePtr newSequence = NewCorrespondenceMaps2DSequence();
	for(int imageIndex = currentRightCameraIndex; imageIndex != currentLeftCameraIndex; imageIndex = (imageIndex - 1) % (parameters.numberOfAdjustedStereoPairs * 2) )
		{
		CorrespondenceMap2DConstPtr newCorrespondence = MatchFeatures( leftFeaturesVector, featuresVectorsList.at(imageIndex) );
		AddCorrespondenceMap(*newSequence, *newCorrespondence);
		}
	for(int imageIndex = mostRecentPastCameraIndex; imageIndex != currentLeftCameraIndex; imageIndex = (imageIndex - 1) % (parameters.numberOfAdjustedStereoPairs * 2) )
		{
		CorrespondenceMap2DConstPtr newCorrespondence = MatchFeatures( rightFeaturesVector, featuresVectorsList.at(imageIndex) );
		AddCorrespondenceMap(*newSequence, *newCorrespondence);
		}

	//Adding the old correspondences to thec current ones. All correspondences are taken except those with the oldest left and right images
	if (0 < currentInputNumber && currentInputNumber < parameters.numberOfAdjustedStereoPairs)
		{
		for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondenceMaps(*latestCorrespondenceMaps); correspondenceIndex++)
			{
			AddCorrespondenceMap(*newSequence, GetCorrespondenceMap(*latestCorrespondenceMaps, correspondenceIndex));
			}
		}
	else if (currentInputNumber > 0);
		{
		int correspondenceIndex = 0;
		for(int sourceIndex = 0; sourceIndex < 2 * (parameters.numberOfAdjustedStereoPairs - 1); sourceIndex++)
			{
			for(int sinkIndex = sourceIndex + 1; sinkIndex < 2 * (parameters.numberOfAdjustedStereoPairs - 1); sinkIndex++)
				{
				AddCorrespondenceMap(*newSequence, GetCorrespondenceMap(*latestCorrespondenceMaps, correspondenceIndex));	
				correspondenceIndex++;
				}
			correspondenceIndex += 2;
			}
		}

	//Updating the latestCorrespondenceMapsSequence
	DELETE_PREVIOUS(latestCorrespondenceMaps);
	latestCorrespondenceMaps = newSequence;
	}

void AdjustmentFromStereo::ExtractFeatures(FrameWrapper::FrameConstPtr filteredImage, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr& keypointsVector)
	{
	featuresExtractor2d->frameInput(*filteredImage);
	featuresExtractor2d->process();

	DELETE_PREVIOUS(keypointsVector);
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

		DELETE_PREVIOUS(featuresVector);
		VisualPointFeatureVector2DPtr newFeaturesVector = NewVisualPointFeatureVector2D();
		Copy(optionalFeaturesDescriptor2d->featuresOutput(), *newFeaturesVector);
		featuresVector = newFeaturesVector;

		DEBUG_PRINT_TO_LOG("Described Current Features", GetNumberOfPoints(*featuresVector) );
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
	return newCorrespondence;
	}

bool AdjustmentFromStereo::ComputeCameraPoses()
	{
	bundleAdjuster->correspondenceMapsSequenceInput(*latestCorrespondenceMaps);
	bundleAdjuster->process();
	bool successOutput = bundleAdjuster->successOutput();
	
	if (successOutput)
		{
		DELETE_PREVIOUS(latestCameraPoses);
		Poses3DSequencePtr newSequence = NewPoses3DSequence();
		Copy(bundleAdjuster->posesSequenceOutput(), *newSequence);
		latestCameraPoses = newSequence;
		DEBUG_PRINT_TO_LOG("Bundle adjustement success", "");
		}
	else
		{
		DEBUG_PRINT_TO_LOG("Bundle adjustement failure", "");
		}

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
	const Pose3D& inversePose = GetPose(*latestCameraPoses, 1);
	
	float newPoseX = GetXPosition(*previousCameraPose) - GetXPosition(inversePose);
	float newPoseY = GetYPosition(*previousCameraPose) - GetYPosition(inversePose);
	float newPoseZ = GetZPosition(*previousCameraPose) - GetZPosition(inversePose);
	float newOrientationX = GetXOrientation(*previousCameraPose) - GetXOrientation(inversePose);
	float newOrientationY = GetYOrientation(*previousCameraPose) - GetYOrientation(inversePose);
	float newOrientationZ = GetZOrientation(*previousCameraPose) - GetZOrientation(inversePose);
	float newOrientationW = GetWOrientation(*previousCameraPose) - GetWOrientation(inversePose);
	float norm = std::sqrt(newOrientationX*newOrientationX + newOrientationY * newOrientationY + newOrientationZ * newOrientationZ + newOrientationW * newOrientationW);

	SetPosition(*previousCameraPose, newPoseX, newPoseY, newPoseZ);
	SetOrientation(*previousCameraPose, newOrientationX/norm, newOrientationY/norm, newOrientationZ/norm, newOrientationW/norm);

	return previousCameraPose;
	}

}



/** @} */
