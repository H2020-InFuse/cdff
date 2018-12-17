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
		DeleteIfNotNull(filteredCurrentLeftImage);
		}
	if (rightFilter != NULL)
		{
		DeleteIfNotNull(filteredCurrentRightImage);
		}
	delete(filteredPastLeftImage);

	if (optionalFeaturesDescriptor != NULL)
		{
		DeleteIfNotNull(currentLeftFeaturesVector);
		DeleteIfNotNull(pastLeftFeaturesVector);
		DeleteIfNotNull(currentRightFeaturesVector);
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

void ReconstructionFromMotion::ComputePointCloud()
	{
	ASSERT(currentLeftFeaturesVector != NULL, "ReconstructionFromMotion error, ComputePointCloud() called before left feature vector was computed");
	FilterCurrentRightImage();
	ExtractCurrentRightFeatures();
	DescribeCurrentRightFeatures();
	MatchLeftAndRightFeatures();
	ComputeStereoPointCloud();
	}

void ReconstructionFromMotion::UpdateScene()
	{
	map->AddFramePoseInReference(pastToCurrentCameraTransform);
	map->AddPointCloudInLastReference(pointCloud);

	Pose3DConstPtr outputPose = map->GetCurrentFramePoseInOrigin();
	PointCloudConstPtr outputPointCloud = map->GetPartialScene(parameters.searchRadius);

	Copy(*outputPose, outPose);
	Copy(*outputPointCloud, outPointCloud);
	DEBUG_PRINT_TO_LOG("Scene Cloud", GetNumberOfPoints(*outputPointCloud));
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
	}

void ReconstructionFromMotion::MatchLeftAndRightFeatures()
	{
	featuresMatcher->sourceFeaturesInput(*currentRightFeaturesVector);
	featuresMatcher->sinkFeaturesInput(*currentLeftFeaturesVector);
	featuresMatcher->process();
	Copy( featuresMatcher->matchesOutput(), *leftRightCorrespondenceMap);	
	DEBUG_PRINT_TO_LOG("Left Right Correspondences", GetNumberOfCorrespondences(*leftRightCorrespondenceMap) );
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
	return essentialMatrixSuccess;
	}

void ReconstructionFromMotion::ComputeStereoPointCloud()
	{
	reconstructor3D->poseInput(*rightToLeftCameraPose);
	reconstructor3D->matchesInput(*leftRightCorrespondenceMap);
	reconstructor3D->process();
	Copy( reconstructor3D->pointcloudOutput(), *pointCloud);
	DEBUG_PRINT_TO_LOG("Point Cloud", GetNumberOfPoints(*pointCloud));
	}

}
}
}


/** @} */
