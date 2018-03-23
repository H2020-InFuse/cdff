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
ReconstructionFromStereo::ReconstructionFromStereo(Map* map) :
	map(map)
	{
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
	leftToRightCorrespondenceMap = NULL;
	fundamentalMatrix = NULL;
	pastToCurrentCameraTransform = NULL;
	pointCloud = NULL;

	leftFilter = NULL;
	rightFilter = NULL;
	featuresExtractor = NULL;
	featuresMatcher = NULL;
	fundamentalMatrixComputer = NULL;
	cameraTransformEstimator = NULL;
	reconstructor3D = NULL;
	optionalFeaturesDescriptor = NULL;

	searchRadius = -1;

	configurationFilePath = "";
	}

ReconstructionFromStereo::~ReconstructionFromStereo()
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
	DELETE_PREVIOUS(leftToRightCorrespondenceMap);
	DELETE_PREVIOUS(fundamentalMatrix);
	DELETE_PREVIOUS(pastToCurrentCameraTransform);
	DELETE_PREVIOUS(pointCloud);
	}

/**
* The process method is split into three steps 
* (i) computation of the camera transform between the current camera pose and the pose of the camera at an appropriate past time instant, the appropriate time instant is the most recent one that allows
* a computation of the transform, if there exists one
* (ii) if a camera transform is computed, then a point cloud is computed from the left and right images of the stereo camera
* (iii) the point cloud rover map is updated with the newly computed point cloud.
*
**/
void ReconstructionFromStereo::run() 
	{
	DEBUG_PRINT_TO_LOG("Structure from stereo start", "");
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

void ReconstructionFromStereo::setup()
	{
	configurator.configure(configurationFilePath);
	AssignDfnsAlias();
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */

/**
* The ComputeCameraMovement Method performs the following operation
*
* (i) Filtering, features extraction and computation of features descriptor applied to the current left camera image;
* (ii) Search of an appropriate past image, this is a linear search on all the past images starting with the most recent one;
*      (ii a) for each past image, there is a filtering, features extraction and computation of features descriptor applied to the past left camera image;
*      (ii b) features matching, computation of the fundamental matrix and extimation of the transform between present and past pose are performed on the base of the extracted features
*      (ii c) the search stops when a transform is successfully estimated or there are no more past images
**/
bool ReconstructionFromStereo::ComputeCameraMovement()
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
* The ComputePointCloud Method works under the assumption that the current left image has been already filtered by a previous processing step.
*
* The method filters the current right image, and uses the filtered current left and right images for the computation of a point cloud.
*
**/
void ReconstructionFromStereo::ComputePointCloud()
	{
	ASSERT(filteredCurrentLeftImage != NULL, "ReconstructionFromStereo error, ComputePointCloud called before filtering the left image");
	FilterCurrentRightImage();
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
void ReconstructionFromStereo::UpdateScene()
	{
	map->AddFramePoseInReference(pastToCurrentCameraTransform);
	map->AddPointCloudInLastReference(pointCloud);

	outPointCloud = map->GetPartialScene(searchRadius);
	outPose = map->GetCurrentFramePoseInOrigin();

	DEBUG_PRINT_TO_LOG("Scene Cloud", GetNumberOfPoints(*outPointCloud));
	DEBUG_SHOW_POINT_CLOUD(outPointCloud);
	}

void ReconstructionFromStereo::AssignDfnsAlias()
	{
	leftFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter") );
	rightFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter") );
	featuresExtractor = static_cast<FeaturesExtraction2DInterface*>( configurator.GetDfn("featureExtractor") );
	featuresMatcher = static_cast<FeaturesMatching2DInterface*>( configurator.GetDfn("featuresMatcher") );
	fundamentalMatrixComputer = static_cast<FundamentalMatrixComputationInterface*>( configurator.GetDfn("fundamentalMatrixComputer") );
	cameraTransformEstimator = static_cast<CamerasTransformEstimationInterface*>( configurator.GetDfn("cameraTransformEstimator") );
	reconstructor3D = static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3D") );
	optionalFeaturesDescriptor = static_cast<FeaturesDescription2DInterface*>( configurator.GetDfn("featuresDescriptor", true) );

	ASSERT(leftFilter != NULL, "DFPC Structure from motion error: left filter DFN configured incorrectly");
	ASSERT(rightFilter != NULL, "DFPC Structure from motion error: right filter DFN configured incorrectly");
	ASSERT(featuresExtractor != NULL, "DFPC Structure from motion error: featuresExtractor DFN configured incorrectly");
	ASSERT(featuresMatcher != NULL, "DFPC Structure from motion error: featuresMatcher DFN configured incorrectly");
	ASSERT(fundamentalMatrixComputer != NULL, "DFPC Structure from motion error: fundamentalMatrixComputer DFN configured incorrectly");
	ASSERT(cameraTransformEstimator != NULL, "DFPC Structure from motion error: cameraTransformEstimator DFN configured incorrectly");
	ASSERT(reconstructor3D != NULL, "DFPC Structure from motion error: reconstructor3D DFN configured incorrectly");
	}

void ReconstructionFromStereo::FilterCurrentLeftImage()
	{
	leftFilter->imageInput(currentLeftImage);
	leftFilter->process();
	DELETE_PREVIOUS(filteredCurrentLeftImage);
	filteredCurrentLeftImage = leftFilter->filteredImageOutput();
	DEBUG_PRINT_TO_LOG("Filtered Current Frame", "");
	DEBUG_SHOW_IMAGE(filteredCurrentLeftImage);
	}

void ReconstructionFromStereo::FilterPastLeftImage()
	{
	leftFilter->imageInput(pastLeftImage);
	leftFilter->process();
	DELETE_PREVIOUS(filteredPastLeftImage);
	filteredPastLeftImage = leftFilter->filteredImageOutput();
	DEBUG_PRINT_TO_LOG("Filtered Past Frame", "");
	}

void ReconstructionFromStereo::FilterCurrentRightImage()
	{
	rightFilter->imageInput(currentRightImage);
	rightFilter->process();
	DELETE_PREVIOUS(filteredCurrentRightImage);
	filteredCurrentRightImage = rightFilter->filteredImageOutput();
	DEBUG_PRINT_TO_LOG("Filtered Current Right Frame", "");
	DEBUG_SHOW_IMAGE(filteredCurrentRightImage);
	}

void ReconstructionFromStereo::ExtractCurrentLeftFeatures()
	{
	featuresExtractor->imageInput(filteredCurrentLeftImage);
	featuresExtractor->process();
	DELETE_PREVIOUS(currentLeftKeypointsVector);
	currentLeftKeypointsVector = featuresExtractor->featuresSetOutput();
	DEBUG_PRINT_TO_LOG("Extracted Current Features", GetNumberOfPoints(*currentLeftKeypointsVector) );
	}

void ReconstructionFromStereo::ExtractPastLeftFeatures()
	{
	featuresExtractor->imageInput(filteredPastLeftImage);
	featuresExtractor->process();
	DELETE_PREVIOUS(pastLeftKeypointsVector);
	pastLeftKeypointsVector = featuresExtractor->featuresSetOutput();
	DEBUG_PRINT_TO_LOG("Extracted Past Features", GetNumberOfPoints(*pastLeftKeypointsVector) );
	}

void ReconstructionFromStereo::ExtractCurrentRightFeatures()
	{
	featuresExtractor->imageInput(filteredCurrentRightImage);
	featuresExtractor->process();
	DELETE_PREVIOUS(currentRightKeypointsVector);
	currentRightKeypointsVector = featuresExtractor->featuresSetOutput();
	DEBUG_PRINT_TO_LOG("Extracted Current Right Features", GetNumberOfPoints(*currentRightKeypointsVector) );
	}

void ReconstructionFromStereo::DescribeCurrentLeftFeatures()
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

void ReconstructionFromStereo::DescribePastLeftFeatures()
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

void ReconstructionFromStereo::DescribeCurrentRightFeatures()
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

void ReconstructionFromStereo::MatchCurrentAndPastFeatures()
	{
	featuresMatcher->sourceFeaturesVectorInput(currentLeftFeaturesVector);
	featuresMatcher->sinkFeaturesVectorInput(pastLeftFeaturesVector);
	featuresMatcher->process();
	DELETE_PREVIOUS(pastToCurrentCorrespondenceMap);
	pastToCurrentCorrespondenceMap = featuresMatcher->correspondenceMapOutput();	
	DEBUG_PRINT_TO_LOG("Correspondences", GetNumberOfCorrespondences(*pastToCurrentCorrespondenceMap) );
	DEBUG_SHOW_2D_CORRESPONDENCES(filteredCurrentLeftImage, filteredPastLeftImage, pastToCurrentCorrespondenceMap);
	}

void ReconstructionFromStereo::MatchLeftAndRightFeatures()
	{
	featuresMatcher->sourceFeaturesVectorInput(currentRightFeaturesVector);
	featuresMatcher->sinkFeaturesVectorInput(currentLeftFeaturesVector);
	featuresMatcher->process();
	DELETE_PREVIOUS(leftToRightCorrespondenceMap);
	leftToRightCorrespondenceMap = featuresMatcher->correspondenceMapOutput();	
	DEBUG_PRINT_TO_LOG("Left Right Correspondences", GetNumberOfCorrespondences(*leftToRightCorrespondenceMap) );
	DEBUG_SHOW_2D_CORRESPONDENCES(filteredCurrentRightImage, filteredCurrentLeftImage, leftToRightCorrespondenceMap);
	}

bool ReconstructionFromStereo::ComputeFundamentalMatrix()
	{
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

bool ReconstructionFromStereo::ComputePastToCurrentTransform()
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

void ReconstructionFromStereo::ComputeStereoPointCloud()
	{
	reconstructor3D->leftImageInput(filteredCurrentLeftImage);
	reconstructor3D->rightImageInput(filteredCurrentRightImage);
	reconstructor3D->process();
	DELETE_PREVIOUS(pointCloud);
	pointCloud = reconstructor3D->pointCloudOutput();
	DEBUG_PRINT_TO_LOG("Point Cloud", GetNumberOfPoints(*pointCloud));
	DEBUG_SHOW_POINT_CLOUD(pointCloud);
	}

}


/** @} */
