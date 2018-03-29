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

	filteredCurrentLeftImage = NULL;
	filteredPastLeftImage = NULL;
	filteredCurrentRightImage = NULL;
	currentLeftKeypointsVector = NULL;
	pastLeftKeypointsVector = NULL;
	currentRightKeypointsVector = NULL;
	currentLeftFeaturesVector = NULL;
	pastLeftFeaturesVector = NULL;
	currentRightFeaturesVector = NULL;
	pastToCurrentCorrespondenceMap = NULL;
	leftRightCorrespondenceMap = NULL;
	fundamentalMatrix = NULL;
	pastToCurrentCameraTransform = NULL;
	pointCloud = NULL;

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
	DELETE_PREVIOUS(filteredCurrentLeftImage);
	DELETE_PREVIOUS(filteredPastLeftImage);
	DELETE_PREVIOUS(filteredCurrentRightImage);
	DELETE_PREVIOUS(currentLeftKeypointsVector);
	DELETE_PREVIOUS(pastLeftKeypointsVector);
	DELETE_PREVIOUS(currentRightKeypointsVector);
	if (optionalFeaturesDescriptor != NULL)
		{
		DELETE_PREVIOUS(currentLeftFeaturesVector);
		DELETE_PREVIOUS(pastLeftFeaturesVector);
		DELETE_PREVIOUS(currentRightFeaturesVector);
		}
	DELETE_PREVIOUS(pastToCurrentCorrespondenceMap);
	DELETE_PREVIOUS(leftRightCorrespondenceMap);
	DELETE_PREVIOUS(fundamentalMatrix);
	DELETE_PREVIOUS(pastToCurrentCameraTransform);
	DELETE_PREVIOUS(pointCloud);
	delete(rightToLeftCameraPose);
	}

/**
* The process method is split into three steps 
* (i) computation of the camera transform between the current camera pose and the pose of the camera at an appropriate past time instant, the appropriate time instant is the most recent one that allows
* a computation of the transform, if there exists one
* (ii) if a camera transform is computed, then a point cloud is computed from the left and right images of the stereo camera
* (iii) the point cloud rover map is updated with the newly computed point cloud.
*
**/
void ReconstructionFromMotion::run() 
	{
	DEBUG_PRINT_TO_LOG("Structure from motion start", "");
	outSuccess = ComputeCameraMovement();

	if (outSuccess)
		{
		ComputePointCloud();
		UpdateScene();
		}
	else
		{
		outPointCloud = NULL;
		outPose = NULL;
		}
	}

void ReconstructionFromMotion::setup()
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
void ReconstructionFromMotion::ConfigureExtraParameters()
	{
	
	parametersHelper.ReadFile( configurator.GetExtraParametersConfigurationFilePath() );

	SetPosition(*rightToLeftCameraPose, parameters.rightToLeftCameraPose.positionX, parameters.rightToLeftCameraPose.positionY, parameters.rightToLeftCameraPose.positionZ);
	SetOrientation(*rightToLeftCameraPose, parameters.rightToLeftCameraPose.orientationX, parameters.rightToLeftCameraPose.orientationY, 
			parameters.rightToLeftCameraPose.orientationZ, parameters.rightToLeftCameraPose.orientationW);
	}

/**
* The ComputeCloudInSight Method performs the following operation
*
* (i) Filtering, features extraction and computation of features descriptor applied to the current left camera image;
* (ii) Search of an appropriate past image, this is a linear search on all the past images starting with the most recent one;
*      (ii a) for each past image, there is a filtering, features extraction and computation of features descriptor applied to the past left camera image;
*      (ii b) features matching, computation of the fundamental matrix and extimation of the transform between present and past pose are performed on the base of the extracted features
*      (ii d) the search stops when a transform is successfully estimated or there are no more past images
**/
bool ReconstructionFromMotion::ComputeCameraMovement()
	{
	currentLeftImage = inLeftImage;
	currentRightImage = inRightImage;
	map->AddFrames(inLeftImage, inRightImage);
	FilterCurrentLeftImage();
	ExtractCurrentLeftFeatures();
	DescribeCurrentLeftFeatures();

	bool success = false;
	for(pastLeftImage = map->GetNextReferenceLeftFrame(); !success && pastLeftImage != NULL; pastLeftImage = map->GetNextReferenceLeftFrame())
		{
		DEBUG_PRINT_TO_LOG("Selected Past Frame", success);
		FilterPastLeftImage();
		ExtractPastLeftFeatures();
		DescribePastLeftFeatures();

		MatchCurrentAndPastFeatures();
		success = ComputeFundamentalMatrix();
		if (success)
			{
			success = ComputePastToCurrentTransform();
			}
		}

	return success;
	}

/**
* The ComputePointCloud Method works under the assumption that the left image features were already extracted.
*
* The method filters executes the following operations: 
* (i) Filtering, features extraction and computation of features descriptor applied to the current right  camera image;
* (ii) Computation of a correspondence map between left and right features;
* (iii) computation of a point cloud from the correspondence map.
*
**/
void ReconstructionFromMotion::ComputePointCloud()
	{
	ASSERT(currentLeftFeaturesVector != NULL, "ReconstructionFromMotion error, ComputePointCloud() called before left feature vector was computed");
	FilterCurrentRightImage();
	ExtractCurrentRightFeatures();
	DescribeCurrentRightFeatures();
	MatchLeftAndRightFeatures();
	ComputeStereoPointCloud();
	}


/**
* The UpdateScene Method performs the following operation
*
* (i) The computed transform between the current and the past images is provided to the Map object, which updates the current pose of the camera;
* (ii) The computed point cloud is provided to the Map object, which updates the whole point cloud map, by adding more cloud points using the current camera location as a reference
* (iii) A part of the point cloud is extracted as output of the DFPC, this part contains the points withing a given searchRadius from the current camera pose, when the search radius is -1
*       all cloud points are taken.
**/
void ReconstructionFromMotion::UpdateScene()
	{
	map->AddFramePoseInReference(pastToCurrentCameraTransform);
	map->AddPointCloudInLastReference(pointCloud);

	outPose = map->GetCurrentFramePoseInOrigin();
	outPointCloud = map->GetPartialScene(searchRadius);

	DEBUG_PRINT_TO_LOG("Scene Cloud", GetNumberOfPoints(*outPointCloud));
	DEBUG_SHOW_POINT_CLOUD(outPointCloud);
	}

void ReconstructionFromMotion::AssignDfnsAlias()
	{
	filter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("filter") );
	featuresExtractor = static_cast<FeaturesExtraction2DInterface*>( configurator.GetDfn("featureExtractor") );
	featuresMatcher = static_cast<FeaturesMatching2DInterface*>( configurator.GetDfn("featuresMatcher") );
	fundamentalMatrixComputer = static_cast<FundamentalMatrixComputationInterface*>( configurator.GetDfn("fundamentalMatrixComputer") );
	cameraTransformEstimator = static_cast<CamerasTransformEstimationInterface*>( configurator.GetDfn("cameraTransformEstimator") );
	reconstructor3D = static_cast<PointCloudReconstruction2DTo3DInterface*>( configurator.GetDfn("reconstructor3D") );
	optionalFeaturesDescriptor = static_cast<FeaturesDescription2DInterface*>( configurator.GetDfn("featuresDescriptor", true) );

	ASSERT(filter != NULL, "DFPC Structure from motion error: filter DFN configured incorrectly");
	ASSERT(featuresExtractor != NULL, "DFPC Structure from motion error: featuresExtractor DFN configured incorrectly");
	ASSERT(featuresMatcher != NULL, "DFPC Structure from motion error: featuresMatcher DFN configured incorrectly");
	ASSERT(fundamentalMatrixComputer != NULL, "DFPC Structure from motion error: fundamentalMatrixComputer DFN configured incorrectly");
	ASSERT(cameraTransformEstimator != NULL, "DFPC Structure from motion error: cameraTransformEstimator DFN configured incorrectly");
	ASSERT(reconstructor3D != NULL, "DFPC Structure from motion error: reconstructor3D DFN configured incorrectly");
	}

void ReconstructionFromMotion::FilterCurrentLeftImage()
	{
	filter->imageInput(currentLeftImage);
	filter->process();
	DELETE_PREVIOUS(filteredCurrentLeftImage);
	filteredCurrentLeftImage = filter->filteredImageOutput();
	DEBUG_PRINT_TO_LOG("Filtered Current Frame", "");
	}

void ReconstructionFromMotion::FilterPastLeftImage()
	{
	filter->imageInput(pastLeftImage);
	filter->process();
	DELETE_PREVIOUS(filteredPastLeftImage);
	filteredPastLeftImage = filter->filteredImageOutput();
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

void ReconstructionFromMotion::ExtractCurrentLeftFeatures()
	{
	featuresExtractor->imageInput(filteredCurrentLeftImage);
	featuresExtractor->process();
	DELETE_PREVIOUS(currentLeftKeypointsVector);
	currentLeftKeypointsVector = featuresExtractor->featuresSetOutput();
	DEBUG_PRINT_TO_LOG("Extracted Current Features", GetNumberOfPoints(*currentLeftKeypointsVector) );
	}

void ReconstructionFromMotion::ExtractPastLeftFeatures()
	{
	featuresExtractor->imageInput(filteredPastLeftImage);
	featuresExtractor->process();
	DELETE_PREVIOUS(pastLeftKeypointsVector);
	pastLeftKeypointsVector = featuresExtractor->featuresSetOutput();
	DEBUG_PRINT_TO_LOG("Extracted Past Features", GetNumberOfPoints(*pastLeftKeypointsVector) );
	}

void ReconstructionFromMotion::ExtractCurrentRightFeatures()
	{
	featuresExtractor->imageInput(filteredCurrentRightImage);
	featuresExtractor->process();
	DELETE_PREVIOUS(currentRightKeypointsVector);
	currentRightKeypointsVector = featuresExtractor->featuresSetOutput();
	DEBUG_PRINT_TO_LOG("Extracted Current Right Features", GetNumberOfPoints(*currentRightKeypointsVector) );
	}

void ReconstructionFromMotion::DescribeCurrentLeftFeatures()
	{
	if (optionalFeaturesDescriptor != NULL)
		{
		optionalFeaturesDescriptor->featuresSetInput(currentLeftKeypointsVector);
		optionalFeaturesDescriptor->process();
		DELETE_PREVIOUS(currentLeftFeaturesVector);
		currentLeftFeaturesVector = optionalFeaturesDescriptor->featuresSetWithDescriptorsOutput();
		DEBUG_PRINT_TO_LOG("Described Current Features", GetNumberOfPoints(*currentLeftFeaturesVector) );
		}
	else
		{
		currentLeftFeaturesVector = currentLeftKeypointsVector;
		}
	}

void ReconstructionFromMotion::DescribePastLeftFeatures()
	{
	if (optionalFeaturesDescriptor != NULL)
		{
		optionalFeaturesDescriptor->featuresSetInput(pastLeftKeypointsVector);
		optionalFeaturesDescriptor->process();
		DELETE_PREVIOUS(pastLeftFeaturesVector);
		pastLeftFeaturesVector = optionalFeaturesDescriptor->featuresSetWithDescriptorsOutput();
		DEBUG_PRINT_TO_LOG("Described Past Features", GetNumberOfPoints(*pastLeftFeaturesVector) );
		}
	else
		{
		pastLeftFeaturesVector = pastLeftKeypointsVector;
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
	featuresMatcher->sourceFeaturesVectorInput(currentLeftFeaturesVector);
	featuresMatcher->sinkFeaturesVectorInput(pastLeftFeaturesVector);
	featuresMatcher->process();
	DELETE_PREVIOUS(pastToCurrentCorrespondenceMap);
	pastToCurrentCorrespondenceMap = featuresMatcher->correspondenceMapOutput();	
	DEBUG_PRINT_TO_LOG("Correspondences", GetNumberOfCorrespondences(*pastToCurrentCorrespondenceMap) );
	DEBUG_SHOW_2D_CORRESPONDENCES(filteredCurrentLeftImage, filteredPastLeftImage, pastToCurrentCorrespondenceMap);
	}

void ReconstructionFromMotion::MatchLeftAndRightFeatures()
	{
	featuresMatcher->sourceFeaturesVectorInput(currentRightFeaturesVector);
	featuresMatcher->sinkFeaturesVectorInput(currentLeftFeaturesVector);
	featuresMatcher->process();
	DELETE_PREVIOUS(leftRightCorrespondenceMap);
	leftRightCorrespondenceMap = featuresMatcher->correspondenceMapOutput();	
	DEBUG_PRINT_TO_LOG("Left Right Correspondences", GetNumberOfCorrespondences(*leftRightCorrespondenceMap) );
	DEBUG_SHOW_2D_CORRESPONDENCES(filteredCurrentRightImage, filteredCurrentLeftImage, leftRightCorrespondenceMap);
	}

bool ReconstructionFromMotion::ComputeFundamentalMatrix()
	{
	if (GetNumberOfCorrespondences(*pastToCurrentCorrespondenceMap) < 8 )
		{
		return false;
		}
	fundamentalMatrixComputer->correspondenceMapInput(pastToCurrentCorrespondenceMap);
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
	cameraTransformEstimator->correspondenceMapInput(pastToCurrentCorrespondenceMap);
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

void ReconstructionFromMotion::ComputeStereoPointCloud()
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
