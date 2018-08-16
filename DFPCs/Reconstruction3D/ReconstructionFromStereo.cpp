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

namespace CDFF
{
namespace DFPC
{
namespace Reconstruction3D
{

using namespace CDFF::DFN;
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
ReconstructionFromStereo::ReconstructionFromStereo(Map* map)
	{
	if (map == NULL)
		{
		this->map = new ObservedScene();
		}
	else
		{
		this->map = map;
		}

	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudMapResolution", parameters.pointCloudMapResolution, DEFAULT_PARAMETERS.pointCloudMapResolution);
	parametersHelper.AddParameter<float>("GeneralParameters", "SearchRadius", parameters.searchRadius, DEFAULT_PARAMETERS.searchRadius);

	currentLeftImage = NewFrame();
	currentRightImage = NewFrame();
	filteredCurrentLeftImage = NULL;
	filteredPastLeftImage = NewFrame();
	filteredCurrentRightImage = NULL;
	currentLeftKeypointsVector = NewVisualPointFeatureVector2D();
	pastLeftKeypointsVector = NewVisualPointFeatureVector2D();
	currentRightKeypointsVector = NewVisualPointFeatureVector2D();
	currentLeftFeaturesVector = NULL;
	pastLeftFeaturesVector = NULL;
	currentRightFeaturesVector = NULL;
	pastToCurrentCorrespondenceMap = NewCorrespondenceMap2D();
	leftToRightCorrespondenceMap = NewCorrespondenceMap2D();
	fundamentalMatrix = NewMatrix3d();
	pastToCurrentCameraTransform = NewPose3D();
	pointCloud = NewPointCloud();

	leftFilter = NULL;
	rightFilter = NULL;
	featuresExtractor = NULL;
	featuresMatcher = NULL;
	fundamentalMatrixComputer = NULL;
	cameraTransformEstimator = NULL;
	reconstructor3D = NULL;
	optionalFeaturesDescriptor = NULL;

	configurationFilePath = "";
	}

ReconstructionFromStereo::~ReconstructionFromStereo()
	{
	if (leftFilter != NULL)
		{
		DELETE_PREVIOUS(filteredCurrentLeftImage);
		}
	if (rightFilter != NULL)
		{
		DELETE_PREVIOUS(filteredCurrentRightImage);
		}
	delete(filteredPastLeftImage);

	if (optionalFeaturesDescriptor != NULL)
		{
		DELETE_PREVIOUS(currentLeftFeaturesVector);
		DELETE_PREVIOUS(pastLeftFeaturesVector);
		DELETE_PREVIOUS(currentRightFeaturesVector);
		}

	delete(currentLeftImage);
	delete(currentRightImage);
	delete(currentLeftKeypointsVector);
	delete(pastLeftKeypointsVector);
	delete(currentRightKeypointsVector);

	delete(pastToCurrentCorrespondenceMap);
	delete(leftToRightCorrespondenceMap);
	delete(fundamentalMatrix);
	delete(pastToCurrentCameraTransform);
	delete(pointCloud);
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
	}

void ReconstructionFromStereo::setup()
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

const ReconstructionFromStereo::ReconstructionFromStereoOptionsSet ReconstructionFromStereo::DEFAULT_PARAMETERS = 
	{
	.searchRadius = -1,
	.pointCloudMapResolution = 1e-2
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
	map->SetPointCloudMapResolution(parameters.pointCloudMapResolution);
	}

void ReconstructionFromStereo::AssignDfnsAlias()
	{
	leftFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) );
	rightFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) );
	featuresExtractor = static_cast<FeaturesExtraction2DInterface*>( configurator.GetDfn("featureExtractor") );
	featuresMatcher = static_cast<FeaturesMatching2DInterface*>( configurator.GetDfn("featuresMatcher") );
	fundamentalMatrixComputer = static_cast<FundamentalMatrixComputationInterface*>( configurator.GetDfn("fundamentalMatrixComputer") );
	cameraTransformEstimator = static_cast<CamerasTransformEstimationInterface*>( configurator.GetDfn("cameraTransformEstimator") );
	reconstructor3D = static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3D") );
	optionalFeaturesDescriptor = static_cast<FeaturesDescription2DInterface*>( configurator.GetDfn("featuresDescriptor", true) );

	ASSERT(featuresExtractor != NULL, "DFPC Structure from motion error: featuresExtractor DFN configured incorrectly");
	ASSERT(featuresMatcher != NULL, "DFPC Structure from motion error: featuresMatcher DFN configured incorrectly");
	ASSERT(fundamentalMatrixComputer != NULL, "DFPC Structure from motion error: fundamentalMatrixComputer DFN configured incorrectly");
	ASSERT(cameraTransformEstimator != NULL, "DFPC Structure from motion error: cameraTransformEstimator DFN configured incorrectly");
	ASSERT(reconstructor3D != NULL, "DFPC Structure from motion error: reconstructor3D DFN configured incorrectly");

	//This data should be initialized only if the relative DFN is used.
	if (leftFilter != NULL)
		{
		filteredCurrentLeftImage = NewFrame();
		}
	if (rightFilter != NULL)
		{
		filteredCurrentRightImage = NewFrame();
		}
	if (optionalFeaturesDescriptor != NULL)
		{
		currentLeftFeaturesVector = NewVisualPointFeatureVector2D();
		pastLeftFeaturesVector = NewVisualPointFeatureVector2D();
		currentRightFeaturesVector = NewVisualPointFeatureVector2D();
		}
	}

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
	Copy(inLeftImage, *currentLeftImage);
	Copy(inRightImage, *currentRightImage);
	map->AddFrames(currentLeftImage, currentRightImage);

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

	Pose3DConstPtr outputPose = map->GetCurrentFramePoseInOrigin();
	PointCloudConstPtr outputPointCloud = map->GetPartialScene(parameters.searchRadius);

	Copy(*outputPose, outPose);
	Copy(*outputPointCloud, outPointCloud);
	DEBUG_PRINT_TO_LOG("Scene Cloud", GetNumberOfPoints(*outputPointCloud));
	DEBUG_SHOW_POINT_CLOUD(outputPointCloud);
	}

void ReconstructionFromStereo::FilterCurrentLeftImage()
	{
	if (leftFilter != NULL)
		{
		leftFilter->imageInput(*currentLeftImage);
		leftFilter->process();
		Copy( leftFilter->imageOutput(), *filteredCurrentLeftImage);
		DEBUG_PRINT_TO_LOG("Filtered Current Frame", "");
		}
	else
		{
		filteredCurrentLeftImage = currentLeftImage;
		}
	}

void ReconstructionFromStereo::FilterPastLeftImage()
	{
	if (leftFilter != NULL)
		{
		leftFilter->imageInput(*pastLeftImage);
		leftFilter->process();
		Copy( leftFilter->imageOutput(), *filteredPastLeftImage);
		DEBUG_PRINT_TO_LOG("Filtered Past Frame", "");
		}
	else
		{
		Copy(*pastLeftImage, *filteredPastLeftImage);
		}
	}

void ReconstructionFromStereo::FilterCurrentRightImage()
	{
	if (rightFilter != NULL)
		{
		rightFilter->imageInput(*currentRightImage);
		rightFilter->process();
		Copy( rightFilter->imageOutput(), *filteredCurrentRightImage);
		DEBUG_PRINT_TO_LOG("Filtered Current Right Frame", "");
		}
	else
		{
		filteredCurrentRightImage = currentRightImage;
		}
	}

void ReconstructionFromStereo::ExtractCurrentLeftFeatures()
	{
	featuresExtractor->frameInput(*filteredCurrentLeftImage);
	featuresExtractor->process();
	Copy( featuresExtractor->featuresOutput(), *currentLeftKeypointsVector);
	DEBUG_PRINT_TO_LOG("Extracted Current Features", GetNumberOfPoints(*currentLeftKeypointsVector) );
	}

void ReconstructionFromStereo::ExtractPastLeftFeatures()
	{
	featuresExtractor->frameInput(*filteredPastLeftImage);
	featuresExtractor->process();
	Copy( featuresExtractor->featuresOutput(), *pastLeftKeypointsVector);
	DEBUG_PRINT_TO_LOG("Extracted Past Features", GetNumberOfPoints(*pastLeftKeypointsVector) );
	}

void ReconstructionFromStereo::ExtractCurrentRightFeatures()
	{
	featuresExtractor->frameInput(*filteredCurrentRightImage);
	featuresExtractor->process();
	Copy( featuresExtractor->featuresOutput(), *currentRightKeypointsVector);
	DEBUG_PRINT_TO_LOG("Extracted Current Right Features", GetNumberOfPoints(*currentRightKeypointsVector) );
	}

void ReconstructionFromStereo::DescribeCurrentLeftFeatures()
	{
	if (optionalFeaturesDescriptor != NULL)
		{
		optionalFeaturesDescriptor->frameInput(*filteredCurrentLeftImage);
		optionalFeaturesDescriptor->featuresInput(*currentLeftKeypointsVector);
		optionalFeaturesDescriptor->process();
		Copy( optionalFeaturesDescriptor->featuresOutput(), *currentLeftFeaturesVector);
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
		optionalFeaturesDescriptor->frameInput(*filteredPastLeftImage);
		optionalFeaturesDescriptor->featuresInput(*pastLeftKeypointsVector);
		optionalFeaturesDescriptor->process();
		Copy(optionalFeaturesDescriptor->featuresOutput(), *pastLeftFeaturesVector);
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
		optionalFeaturesDescriptor->frameInput(*filteredCurrentRightImage);
		optionalFeaturesDescriptor->featuresInput(*currentRightKeypointsVector);
		optionalFeaturesDescriptor->process();
		Copy( optionalFeaturesDescriptor->featuresOutput(), *currentRightFeaturesVector);
		DEBUG_PRINT_TO_LOG("Described Current Right Features", GetNumberOfPoints(*currentRightFeaturesVector) );
		}
	else
		{
		currentRightFeaturesVector = currentRightKeypointsVector;
		}
	}

void ReconstructionFromStereo::MatchCurrentAndPastFeatures()
	{
	featuresMatcher->sourceFeaturesInput(*currentLeftFeaturesVector);
	featuresMatcher->sinkFeaturesInput(*pastLeftFeaturesVector);
	featuresMatcher->process();
	Copy( featuresMatcher->matchesOutput(), *pastToCurrentCorrespondenceMap);	
	DEBUG_PRINT_TO_LOG("Correspondences", GetNumberOfCorrespondences(*pastToCurrentCorrespondenceMap) );
	DEBUG_SHOW_2D_CORRESPONDENCES(filteredCurrentLeftImage, filteredPastLeftImage, pastToCurrentCorrespondenceMap);
	}

void ReconstructionFromStereo::MatchLeftAndRightFeatures()
	{
	featuresMatcher->sourceFeaturesInput(*currentRightFeaturesVector);
	featuresMatcher->sinkFeaturesInput(*currentLeftFeaturesVector);
	featuresMatcher->process();
	Copy( featuresMatcher->matchesOutput(), *leftToRightCorrespondenceMap);	
	DEBUG_PRINT_TO_LOG("Left Right Correspondences", GetNumberOfCorrespondences(*leftToRightCorrespondenceMap) );
	DEBUG_SHOW_2D_CORRESPONDENCES(filteredCurrentRightImage, filteredCurrentLeftImage, leftToRightCorrespondenceMap);
	}

bool ReconstructionFromStereo::ComputeFundamentalMatrix()
	{
	if (GetNumberOfCorrespondences(*pastToCurrentCorrespondenceMap) < 8 )
		{
		return false;
		}
	fundamentalMatrixComputer->matchesInput(*pastToCurrentCorrespondenceMap);
	fundamentalMatrixComputer->process();
	Copy( fundamentalMatrixComputer->fundamentalMatrixOutput(), *fundamentalMatrix);	
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
	cameraTransformEstimator->fundamentalMatrixInput(*fundamentalMatrix);
	cameraTransformEstimator->matchesInput(*pastToCurrentCorrespondenceMap);
	cameraTransformEstimator->process();
	Copy( cameraTransformEstimator->transformOutput(), *pastToCurrentCameraTransform);
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
	reconstructor3D->leftInput(*filteredCurrentLeftImage);
	reconstructor3D->rightInput(*filteredCurrentRightImage);
	reconstructor3D->process();
	Copy( reconstructor3D->pointcloudOutput(), *pointCloud);
	DEBUG_PRINT_TO_LOG("Point Cloud", GetNumberOfPoints(*pointCloud));
	DEBUG_SHOW_POINT_CLOUD(pointCloud);
	}

}
}
}


/** @} */
