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
#include <Visualizers/OpenCVVisualizer.hpp>
#include <Visualizers/PCLVisualizer.hpp>

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
ReconstructionFromMotion::ReconstructionFromMotion(Map* map)
	{
	if (map == NULL)
		{
		this->map = new ObservedScene();
		}
	else
		{
		this->map = map;
		}
	parameters = DEFAULT_PARAMETERS;

	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "PositionX", parameters.rightToLeftCameraPose.positionX, DEFAULT_PARAMETERS.rightToLeftCameraPose.positionX);
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "PositionY", parameters.rightToLeftCameraPose.positionY, DEFAULT_PARAMETERS.rightToLeftCameraPose.positionY);
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "PositionZ", parameters.rightToLeftCameraPose.positionZ, DEFAULT_PARAMETERS.rightToLeftCameraPose.positionZ);

	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "OrientationX", parameters.rightToLeftCameraPose.orientationX, DEFAULT_PARAMETERS.rightToLeftCameraPose.orientationX);
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "OrientationY", parameters.rightToLeftCameraPose.orientationY, DEFAULT_PARAMETERS.rightToLeftCameraPose.orientationY);
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "OrientationZ", parameters.rightToLeftCameraPose.orientationZ, DEFAULT_PARAMETERS.rightToLeftCameraPose.orientationZ);
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "OrientationW", parameters.rightToLeftCameraPose.orientationW, DEFAULT_PARAMETERS.rightToLeftCameraPose.orientationW);

	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudMapResolution", parameters.pointCloudMapResolution, DEFAULT_PARAMETERS.pointCloudMapResolution);
	parametersHelper.AddParameter<float>("GeneralParameters", "SearchRadius", parameters.searchRadius, DEFAULT_PARAMETERS.searchRadius);

	currentLeftImage = NewFrame();
	currentRightImage = NewFrame();
	pastLeftImage = NULL;
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
	leftRightCorrespondenceMap = NewCorrespondenceMap2D();
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

	rightToLeftCameraPose = new Pose3D();
	}

ReconstructionFromMotion::~ReconstructionFromMotion()
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
	delete(leftRightCorrespondenceMap);
	delete(fundamentalMatrix);
	delete(pastToCurrentCameraTransform);
	delete(pointCloud);
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
	/*.searchRadius =*/ -1,
	/*.pointCloudMapResolution =*/ 1e-2,
	//.rightToLeftCameraPose = 
		{
		/*.positionX =*/ 0.122,
		/*.positionY =*/ 0,
		/*.positionZ =*/ 0,
		/*.orientationX =*/ 0,
		/*.orientationY =*/ 0,
		/*.orientationZ =*/ 0,
		/*.orientationW =*/ 1
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

	ASSERT(parameters.pointCloudMapResolution > 0, "RegistrationFromStereo Error, Point Cloud Map resolution is not positive");
	map->SetPointCloudMapResolution(parameters.pointCloudMapResolution);
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

	Pose3DConstPtr outputPose = map->GetCurrentFramePoseInOrigin();
	PointCloudConstPtr outputPointCloud = map->GetPartialScene(parameters.searchRadius);

	Copy(*outputPose, outPose);
	Copy(*outputPointCloud, outPointCloud);
	DEBUG_PRINT_TO_LOG("Scene Cloud", GetNumberOfPoints(*outputPointCloud));
	DEBUG_SHOW_POINT_CLOUD(outputPointCloud);
	}

void ReconstructionFromMotion::AssignDfnsAlias()
	{
	leftFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) );
	rightFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) );
	featuresExtractor = static_cast<FeaturesExtraction2DInterface*>( configurator.GetDfn("featureExtractor") );
	featuresMatcher = static_cast<FeaturesMatching2DInterface*>( configurator.GetDfn("featuresMatcher") );
	fundamentalMatrixComputer = static_cast<FundamentalMatrixComputationInterface*>( configurator.GetDfn("fundamentalMatrixComputer") );
	cameraTransformEstimator = static_cast<CamerasTransformEstimationInterface*>( configurator.GetDfn("cameraTransformEstimator") );
	reconstructor3D = static_cast<PointCloudReconstruction2DTo3DInterface*>( configurator.GetDfn("reconstructor3D") );
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

void ReconstructionFromMotion::FilterCurrentLeftImage()
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

void ReconstructionFromMotion::FilterPastLeftImage()
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

void ReconstructionFromMotion::FilterCurrentRightImage()
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

void ReconstructionFromMotion::ExtractCurrentLeftFeatures()
	{
	featuresExtractor->frameInput(*filteredCurrentLeftImage);
	featuresExtractor->process();
	Copy( featuresExtractor->featuresOutput(), *currentLeftKeypointsVector);
	DEBUG_PRINT_TO_LOG("Extracted Current Features", GetNumberOfPoints(*currentLeftKeypointsVector) );
	}

void ReconstructionFromMotion::ExtractPastLeftFeatures()
	{
	featuresExtractor->frameInput(*filteredPastLeftImage);
	featuresExtractor->process();
	Copy( featuresExtractor->featuresOutput(), *pastLeftKeypointsVector);
	DEBUG_PRINT_TO_LOG("Extracted Past Features", GetNumberOfPoints(*pastLeftKeypointsVector) );
	}

void ReconstructionFromMotion::ExtractCurrentRightFeatures()
	{
	featuresExtractor->frameInput(*filteredCurrentRightImage);
	featuresExtractor->process();
	Copy( featuresExtractor->featuresOutput(), *currentRightKeypointsVector);
	DEBUG_PRINT_TO_LOG("Extracted Current Right Features", GetNumberOfPoints(*currentRightKeypointsVector) );
	}

void ReconstructionFromMotion::DescribeCurrentLeftFeatures()
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

void ReconstructionFromMotion::DescribePastLeftFeatures()
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

void ReconstructionFromMotion::DescribeCurrentRightFeatures()
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

void ReconstructionFromMotion::MatchCurrentAndPastFeatures()
	{
	featuresMatcher->sourceFeaturesInput(*currentLeftFeaturesVector);
	featuresMatcher->sinkFeaturesInput(*pastLeftFeaturesVector);
	featuresMatcher->process();
	Copy( featuresMatcher->matchesOutput(), *pastToCurrentCorrespondenceMap);	
	DEBUG_PRINT_TO_LOG("Correspondences", GetNumberOfCorrespondences(*pastToCurrentCorrespondenceMap) );
	DEBUG_SHOW_2D_CORRESPONDENCES(filteredCurrentLeftImage, filteredPastLeftImage, pastToCurrentCorrespondenceMap);
	}

void ReconstructionFromMotion::MatchLeftAndRightFeatures()
	{
	featuresMatcher->sourceFeaturesInput(*currentRightFeaturesVector);
	featuresMatcher->sinkFeaturesInput(*currentLeftFeaturesVector);
	featuresMatcher->process();
	Copy( featuresMatcher->matchesOutput(), *leftRightCorrespondenceMap);	
	DEBUG_PRINT_TO_LOG("Left Right Correspondences", GetNumberOfCorrespondences(*leftRightCorrespondenceMap) );
	DEBUG_SHOW_2D_CORRESPONDENCES(filteredCurrentRightImage, filteredCurrentLeftImage, leftRightCorrespondenceMap);
	}

bool ReconstructionFromMotion::ComputeFundamentalMatrix()
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

bool ReconstructionFromMotion::ComputePastToCurrentTransform()
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

void ReconstructionFromMotion::ComputeStereoPointCloud()
	{
	reconstructor3D->poseInput(*rightToLeftCameraPose);
	reconstructor3D->matchesInput(*leftRightCorrespondenceMap);
	reconstructor3D->process();
	Copy( reconstructor3D->pointcloudOutput(), *pointCloud);
	DEBUG_PRINT_TO_LOG("Point Cloud", GetNumberOfPoints(*pointCloud));
	DEBUG_SHOW_POINT_CLOUD(pointCloud);
	}

}
}
}


/** @} */
