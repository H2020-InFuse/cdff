/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ReconstructionFromMotion.cpp
 * @date 08/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the ReconstructionFromMotion class.
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
#include "ReconstructionFromMotion.hpp"
#include "Errors/Assert.hpp"
#include <Visualizers/OpencvVisualizer.hpp>
#include <Visualizers/PclVisualizer.hpp>

#define DELETE_PREVIOUS(object) \
	{ \
	if (object != NULL) \
		{ \
		delete(object); \
		} \
	} \

namespace dfpc_ci {

using namespace dfn_ci;
using namespace VisualPointFeatureVector2DWrapper;
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
ReconstructionFromMotion::ReconstructionFromMotion(Map* map) :
	map(map)
	{
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "PositionX", parameters.rightToLeftCameraPose.positionX, DEFAULT_PARAMETERS.rightToLeftCameraPose.positionX);
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "PositionY", parameters.rightToLeftCameraPose.positionY, DEFAULT_PARAMETERS.rightToLeftCameraPose.positionY);
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "PositionZ", parameters.rightToLeftCameraPose.positionZ, DEFAULT_PARAMETERS.rightToLeftCameraPose.positionZ);

	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "OrientationX", parameters.rightToLeftCameraPose.orientationX, DEFAULT_PARAMETERS.rightToLeftCameraPose.orientationX);
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "OrientationY", parameters.rightToLeftCameraPose.orientationY, DEFAULT_PARAMETERS.rightToLeftCameraPose.orientationY);
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "OrientationZ", parameters.rightToLeftCameraPose.orientationZ, DEFAULT_PARAMETERS.rightToLeftCameraPose.orientationZ);
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "OrientationW", parameters.rightToLeftCameraPose.orientationW, DEFAULT_PARAMETERS.rightToLeftCameraPose.orientationW);

	filteredCurrentImage = NULL;
	filteredPastImage = NULL;
	filteredCurrentRightImage = NULL;
	currentKeypointsVector = NULL;
	pastKeypointsVector = NULL;
	currentRightKeypointsVector = NULL;
	currentFeaturesVector = NULL;
	pastFeaturesVector = NULL;
	currentRightFeaturesVector = NULL;
	correspondenceMap = NULL;
	leftRightCorrespondenceMap = NULL;
	fundamentalMatrix = NULL;
	pastToCurrentCameraTransform = NULL;
	pointCloud = NULL;
	sceneCloud = NULL;

	filter = NULL;
	featuresExtractor = NULL;
	featuresMatcher = NULL;
	fundamentalMatrixComputer = NULL;
	cameraTransformEstimator = NULL;
	reconstructor3D = NULL;
	optionalFeaturesDescriptor = NULL;

	searchRadius = -1;

	configurationFilePath = "";

	rightToLeftCameraPose = new Pose3D();
	}

ReconstructionFromMotion::~ReconstructionFromMotion()
	{
	DELETE_PREVIOUS(filteredCurrentImage);
	DELETE_PREVIOUS(filteredPastImage);
	DELETE_PREVIOUS(filteredCurrentRightImage);
	DELETE_PREVIOUS(currentKeypointsVector);
	DELETE_PREVIOUS(pastKeypointsVector);
	DELETE_PREVIOUS(currentRightKeypointsVector);
	if (optionalFeaturesDescriptor != NULL)
		{
		DELETE_PREVIOUS(currentFeaturesVector);
		DELETE_PREVIOUS(pastFeaturesVector);
		DELETE_PREVIOUS(currentRightFeaturesVector);
		}
	DELETE_PREVIOUS(correspondenceMap);
	DELETE_PREVIOUS(leftRightCorrespondenceMap);
	DELETE_PREVIOUS(fundamentalMatrix);
	DELETE_PREVIOUS(pastToCurrentCameraTransform);
	DELETE_PREVIOUS(pointCloud);
	DELETE_PREVIOUS(sceneCloud);
	delete(rightToLeftCameraPose);
	}

void ReconstructionFromMotion::process() 
	{
	DEBUG_PRINT_TO_LOG("Structure from motion start", "");
	outSuccess = ComputeCloudInSight();

	if (outSuccess)
		{
		UpdateScene();
		outPose = map->GetCurrentFramePoseInOrigin();
		outPointCloud = sceneCloud;
		}
	else
		{
		outPointCloud = NULL;
		outPose = NULL;
		}
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */

const ReconstructionFromMotion::ReconstructionFromMotionOptionsSet ReconstructionFromMotion::DEFAULT_PARAMETERS = 
	{
	.rightToLeftCameraPose = 
		{
		.positionX = 0.122,
		.positionY = 0,
		.positionZ = 0,
		.orientationX = 0,
		.orientationY = 0,
		.orientationZ = 0,
		.orientationW = 1
		}
	};

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void ReconstructionFromMotion::ConfigureChain()
	{
	parametersHelper.ReadFile(chainConfigurationFilePath);

	SetPosition(*rightToLeftCameraPose, parameters.rightToLeftCameraPose.positionX, parameters.rightToLeftCameraPose.positionY, parameters.rightToLeftCameraPose.positionZ);
	SetOrientation(*rightToLeftCameraPose, parameters.rightToLeftCameraPose.orientationX, parameters.rightToLeftCameraPose.orientationY, 
			parameters.rightToLeftCameraPose.orientationZ, parameters.rightToLeftCameraPose.orientationW);
	}

bool ReconstructionFromMotion::ComputeCloudInSight()
	{
	currentImage = inLeftImage;
	currentRightImage = inRightImage;
	map->AddFrames(inLeftImage, inRightImage);
	FilterCurrentImage();
	ExtractCurrentFeatures();
	DescribeCurrentFeatures();

	bool success = false;
	for(pastImage = map->GetNextReferenceLeftFrame(); !success && pastImage != NULL; pastImage = map->GetNextReferenceLeftFrame())
		{
		DEBUG_PRINT_TO_LOG("Selected Past Frame", success);
		FilterPastImage();
		ExtractPastFeatures();
		DescribePastFeatures();

		MatchCurrentAndPastFeatures();
		success = ComputeFundamentalMatrix();
		if (success)
			{
			success = ComputePastToCurrentTransform();
			}
		if (success)
			{	
			FilterCurrentRightImage();
			ExtractCurrentRightFeatures();
			DescribeCurrentRightFeatures();
			MatchLeftAndRightFeatures();
			ComputePointCloud();	
			}
		}

	return success;
	}

void ReconstructionFromMotion::UpdateScene()
	{
	map->AddFramePoseInReference(pastToCurrentCameraTransform);
	map->AddPointCloudInLastReference(pointCloud);
	DELETE_PREVIOUS(sceneCloud);
	sceneCloud = map->GetPartialScene(searchRadius);
	DEBUG_PRINT_TO_LOG("Scene Cloud", GetNumberOfPoints(*sceneCloud));
	DEBUG_SHOW_POINT_CLOUD(sceneCloud);
	}

void ReconstructionFromMotion::AssignDfnsAlias()
	{
	unsigned dfnNumber = dfnsSet.size();
	unsigned mandatoryDFNsNumber = 6;
	unsigned optionalDFNsSet = 0;

	filter = static_cast<ImageFilteringInterface*>(dfnsSet["filter"]);
	featuresExtractor = static_cast<FeaturesExtraction2DInterface*>(dfnsSet["featureExtractor"]);
	featuresMatcher = static_cast<FeaturesMatching2DInterface*>(dfnsSet["featuresMatcher"]);
	fundamentalMatrixComputer = static_cast<FundamentalMatrixComputationInterface*>(dfnsSet["fundamentalMatrixComputer"]);
	cameraTransformEstimator = static_cast<CamerasTransformEstimationInterface*>(dfnsSet["cameraTransformEstimator"]);
	reconstructor3D = static_cast<PointCloudReconstruction2DTo3DInterface*>(dfnsSet["reconstructor3D"]);

	if (dfnsSet.find("featuresDescriptor") != dfnsSet.end() )
		{
		optionalFeaturesDescriptor = static_cast<FeaturesDescription2DInterface*>(dfnsSet["featuresDescriptor"]);
		ASSERT(optionalFeaturesDescriptor != NULL, "DFPC Structure from motion error: featuresDescriptor DFN configured incorrectly");
		optionalDFNsSet++;
		}
	else
		{
		optionalFeaturesDescriptor = NULL;
		}

	ASSERT(dfnNumber == mandatoryDFNsNumber + optionalDFNsSet, "DFPC Structure from motion error: wrong number of DFNs in configuration file");
	ASSERT(filter != NULL, "DFPC Structure from motion error: filter DFN configured incorrectly");
	ASSERT(featuresExtractor != NULL, "DFPC Structure from motion error: featuresExtractor DFN configured incorrectly");
	ASSERT(featuresMatcher != NULL, "DFPC Structure from motion error: featuresMatcher DFN configured incorrectly");
	ASSERT(fundamentalMatrixComputer != NULL, "DFPC Structure from motion error: fundamentalMatrixComputer DFN configured incorrectly");
	ASSERT(cameraTransformEstimator != NULL, "DFPC Structure from motion error: cameraTransformEstimator DFN configured incorrectly");
	ASSERT(reconstructor3D != NULL, "DFPC Structure from motion error: reconstructor3D DFN configured incorrectly");
	}

void ReconstructionFromMotion::FilterCurrentImage()
	{
	filter->imageInput(currentImage);
	filter->process();
	DELETE_PREVIOUS(filteredCurrentImage);
	filteredCurrentImage = filter->filteredImageOutput();
	DEBUG_PRINT_TO_LOG("Filtered Current Frame", "");
	}

void ReconstructionFromMotion::FilterPastImage()
	{
	filter->imageInput(pastImage);
	filter->process();
	DELETE_PREVIOUS(filteredPastImage);
	filteredPastImage = filter->filteredImageOutput();
	DEBUG_PRINT_TO_LOG("Filtered Past Frame", "");
	}

void ReconstructionFromMotion::FilterCurrentRightImage()
	{
	filter->imageInput(currentRightImage);
	filter->process();
	DELETE_PREVIOUS(filteredCurrentRightImage);
	filteredCurrentRightImage = filter->filteredImageOutput();
	DEBUG_PRINT_TO_LOG("Filtered Current Right Frame", "");
	}

void ReconstructionFromMotion::ExtractCurrentFeatures()
	{
	featuresExtractor->imageInput(filteredCurrentImage);
	featuresExtractor->process();
	DELETE_PREVIOUS(currentKeypointsVector);
	currentKeypointsVector = featuresExtractor->featuresSetOutput();
	DEBUG_PRINT_TO_LOG("Extracted Current Features", GetNumberOfPoints(*currentKeypointsVector) );
	}

void ReconstructionFromMotion::ExtractPastFeatures()
	{
	featuresExtractor->imageInput(filteredPastImage);
	featuresExtractor->process();
	DELETE_PREVIOUS(pastKeypointsVector);
	pastKeypointsVector = featuresExtractor->featuresSetOutput();
	DEBUG_PRINT_TO_LOG("Extracted Past Features", GetNumberOfPoints(*pastKeypointsVector) );
	}

void ReconstructionFromMotion::ExtractCurrentRightFeatures()
	{
	featuresExtractor->imageInput(filteredCurrentRightImage);
	featuresExtractor->process();
	DELETE_PREVIOUS(currentRightKeypointsVector);
	currentRightKeypointsVector = featuresExtractor->featuresSetOutput();
	DEBUG_PRINT_TO_LOG("Extracted Current Right Features", GetNumberOfPoints(*currentRightKeypointsVector) );
	}

void ReconstructionFromMotion::DescribeCurrentFeatures()
	{
	if (optionalFeaturesDescriptor != NULL)
		{
		optionalFeaturesDescriptor->featuresSetInput(currentKeypointsVector);
		optionalFeaturesDescriptor->process();
		DELETE_PREVIOUS(currentFeaturesVector);
		currentFeaturesVector = optionalFeaturesDescriptor->featuresSetWithDescriptorsOutput();
		DEBUG_PRINT_TO_LOG("Described Current Features", GetNumberOfPoints(*currentFeaturesVector) );
		}
	else
		{
		currentFeaturesVector = currentKeypointsVector;
		}
	}

void ReconstructionFromMotion::DescribePastFeatures()
	{
	if (optionalFeaturesDescriptor != NULL)
		{
		optionalFeaturesDescriptor->featuresSetInput(pastKeypointsVector);
		optionalFeaturesDescriptor->process();
		DELETE_PREVIOUS(pastFeaturesVector);
		pastFeaturesVector = optionalFeaturesDescriptor->featuresSetWithDescriptorsOutput();
		DEBUG_PRINT_TO_LOG("Described Past Features", GetNumberOfPoints(*pastFeaturesVector) );
		}
	else
		{
		pastFeaturesVector = pastKeypointsVector;
		}
	}

void ReconstructionFromMotion::DescribeCurrentRightFeatures()
	{
	if (optionalFeaturesDescriptor != NULL)
		{
		optionalFeaturesDescriptor->featuresSetInput(currentRightKeypointsVector);
		optionalFeaturesDescriptor->process();
		DELETE_PREVIOUS(currentRightFeaturesVector);
		currentRightFeaturesVector = optionalFeaturesDescriptor->featuresSetWithDescriptorsOutput();
		DEBUG_PRINT_TO_LOG("Described Current Right Features", GetNumberOfPoints(*currentRightFeaturesVector) );
		}
	else
		{
		currentRightFeaturesVector = currentRightKeypointsVector;
		}
	}

void ReconstructionFromMotion::MatchCurrentAndPastFeatures()
	{
	featuresMatcher->sourceFeaturesVectorInput(currentFeaturesVector);
	featuresMatcher->sinkFeaturesVectorInput(pastFeaturesVector);
	featuresMatcher->process();
	DELETE_PREVIOUS(correspondenceMap);
	correspondenceMap = featuresMatcher->correspondenceMapOutput();	
	DEBUG_PRINT_TO_LOG("Correspondences", GetNumberOfCorrespondences(*correspondenceMap) );
	DEBUG_SHOW_2D_CORRESPONDENCES(filteredCurrentImage, filteredPastImage, correspondenceMap);
	}

void ReconstructionFromMotion::MatchLeftAndRightFeatures()
	{
	featuresMatcher->sourceFeaturesVectorInput(currentRightFeaturesVector);
	featuresMatcher->sinkFeaturesVectorInput(currentFeaturesVector);
	featuresMatcher->process();
	DELETE_PREVIOUS(leftRightCorrespondenceMap);
	leftRightCorrespondenceMap = featuresMatcher->correspondenceMapOutput();	
	DEBUG_PRINT_TO_LOG("Left Right Correspondences", GetNumberOfCorrespondences(*leftRightCorrespondenceMap) );
	DEBUG_SHOW_2D_CORRESPONDENCES(filteredCurrentRightImage, filteredCurrentImage, leftRightCorrespondenceMap);
	}

bool ReconstructionFromMotion::ComputeFundamentalMatrix()
	{
	fundamentalMatrixComputer->correspondenceMapInput(correspondenceMap);
	fundamentalMatrixComputer->process();
	DELETE_PREVIOUS(fundamentalMatrix);	
	fundamentalMatrix = fundamentalMatrixComputer->fundamentalMatrixOutput();	
	bool fundamentalMatrixSuccess =  fundamentalMatrixComputer->successOutput();
	DEBUG_PRINT_TO_LOG("Fundamental Matrix", fundamentalMatrixSuccess);
	if(fundamentalMatrixSuccess)
		{
		DEBUG_SHOW_MATRIX(fundamentalMatrix);
		}
	return fundamentalMatrixSuccess;
	}

bool ReconstructionFromMotion::ComputePastToCurrentTransform()
	{
	cameraTransformEstimator->fundamentalMatrixInput(fundamentalMatrix);
	cameraTransformEstimator->correspondenceMapInput(correspondenceMap);
	cameraTransformEstimator->process();
	DELETE_PREVIOUS(pastToCurrentCameraTransform);
	pastToCurrentCameraTransform = cameraTransformEstimator->transformOutput();
	bool essentialMatrixSuccess = cameraTransformEstimator->successOutput();
	DEBUG_PRINT_TO_LOG("Essential Matrix", essentialMatrixSuccess);
	if(essentialMatrixSuccess)
		{
		DEBUG_SHOW_POSE(pastToCurrentCameraTransform);
		}
	return essentialMatrixSuccess;
	}

void ReconstructionFromMotion::ComputePointCloud()
	{
	reconstructor3D->poseInput(rightToLeftCameraPose);
	reconstructor3D->correspondenceMapInput(leftRightCorrespondenceMap);
	reconstructor3D->process();
	DELETE_PREVIOUS(pointCloud);
	pointCloud = reconstructor3D->pointCloudOutput();
	DEBUG_PRINT_TO_LOG("Point Cloud", GetNumberOfPoints(*pointCloud));
	DEBUG_SHOW_POINT_CLOUD(pointCloud);
	}

}


/** @} */
