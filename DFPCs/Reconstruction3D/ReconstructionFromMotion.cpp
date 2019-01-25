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
#include <Errors/Assert.hpp>
#include <Errors/AssertOnTest.hpp>

#include <Executors/ImageFiltering/ImageFilteringExecutor.hpp>
#include <Executors/PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DExecutor.hpp>
#include <Executors/FeaturesExtraction2D/FeaturesExtraction2DExecutor.hpp>
#include <Executors/FeaturesDescription2D/FeaturesDescription2DExecutor.hpp>
#include <Executors/FeaturesMatching2D/FeaturesMatching2DExecutor.hpp>
#include <Executors/FundamentalMatrixComputation/FundamentalMatrixComputationExecutor.hpp>
#include <Executors/CamerasTransformEstimation/CamerasTransformEstimationExecutor.hpp>
#include <Executors/PointCloudAssembly/PointCloudAssemblyExecutor.hpp>
#include <Executors/PointCloudTransformation/PointCloudTransformationExecutor.hpp>

namespace CDFF
{
namespace DFPC
{
namespace Reconstruction3D
{

using namespace CDFF::DFN;
using namespace VisualPointFeatureVector2DWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace FrameWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace MatrixWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace Executors;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ReconstructionFromMotion::ReconstructionFromMotion() :
	EMPTY_FEATURE_VECTOR( NewVisualPointFeatureVector3D() ),
	LEFT_FEATURE_CATEGORY ( "left_features" )
	{
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "PositionX", parameters.rightToLeftCameraPose.positionX, DEFAULT_PARAMETERS.rightToLeftCameraPose.positionX);
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "PositionY", parameters.rightToLeftCameraPose.positionY, DEFAULT_PARAMETERS.rightToLeftCameraPose.positionY);
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "PositionZ", parameters.rightToLeftCameraPose.positionZ, DEFAULT_PARAMETERS.rightToLeftCameraPose.positionZ);

	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "OrientationX", parameters.rightToLeftCameraPose.orientationX, DEFAULT_PARAMETERS.rightToLeftCameraPose.orientationX);
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "OrientationY", parameters.rightToLeftCameraPose.orientationY, DEFAULT_PARAMETERS.rightToLeftCameraPose.orientationY);
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "OrientationZ", parameters.rightToLeftCameraPose.orientationZ, DEFAULT_PARAMETERS.rightToLeftCameraPose.orientationZ);
	parametersHelper.AddParameter<float>("RightToLeftCameraPose", "OrientationW", parameters.rightToLeftCameraPose.orientationW, DEFAULT_PARAMETERS.rightToLeftCameraPose.orientationW);

	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudMapResolution", parameters.pointCloudMapResolution, DEFAULT_PARAMETERS.pointCloudMapResolution);
	parametersHelper.AddParameter<float>("GeneralParameters", "SearchRadius", parameters.searchRadius, DEFAULT_PARAMETERS.searchRadius);
	parametersHelper.AddParameter<int>("GeneralParameters", "TrackedHistorySize", parameters.trackedHistorySize, DEFAULT_PARAMETERS.trackedHistorySize);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseAssemblerDfn", parameters.useAssemblerDfn, DEFAULT_PARAMETERS.useAssemblerDfn);

	leftFilter = NULL;
	rightFilter = NULL;
	featuresExtractor = NULL;
	featuresMatcher = NULL;
	fundamentalMatrixComputer = NULL;
	cameraTransformEstimator = NULL;
	reconstructor3D = NULL;
	optionalFeaturesDescriptor = NULL;
	cloudAssembler = NULL;
	cloudTransformer = NULL;

	configurationFilePath = "";

	bundleHistory = NULL;
	firstInput = true;
	SetPosition(zeroPose, 0, 0, 0);
	SetOrientation(zeroPose, 0, 0, 0, 1);
	}

ReconstructionFromMotion::~ReconstructionFromMotion()
	{
	DeleteIfNotNull(leftFilter);
	DeleteIfNotNull(rightFilter);
	DeleteIfNotNull(featuresExtractor);
	DeleteIfNotNull(featuresMatcher);
	DeleteIfNotNull(fundamentalMatrixComputer);
	DeleteIfNotNull(cameraTransformEstimator);
	DeleteIfNotNull(reconstructor3D);
	DeleteIfNotNull(optionalFeaturesDescriptor);
	DeleteIfNotNull(cloudAssembler);
	DeleteIfNotNull(cloudTransformer);
	DeleteIfNotNull(bundleHistory);

	delete(EMPTY_FEATURE_VECTOR);
	}

void ReconstructionFromMotion::run() 
	{
	DEBUG_PRINT_TO_LOG("Structure from motion start", "");
	outSuccess = ComputeCameraMovement();

	if (outSuccess || firstInput)
		{
		PointCloudConstPtr pointCloud = ComputePointCloud();
		UpdateScene(pointCloud);
		firstInput = false;
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
		},
	/*.trackedHistorySize =*/ 5,
	/*.useAssemblerDfn =*/ true
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

	SetPosition(rightToLeftCameraPose, parameters.rightToLeftCameraPose.positionX, parameters.rightToLeftCameraPose.positionY, parameters.rightToLeftCameraPose.positionZ);
	SetOrientation(rightToLeftCameraPose, parameters.rightToLeftCameraPose.orientationX, parameters.rightToLeftCameraPose.orientationY, 
			parameters.rightToLeftCameraPose.orientationZ, parameters.rightToLeftCameraPose.orientationW);

	DeleteIfNotNull(bundleHistory);
	bundleHistory = new BundleHistory(parameters.trackedHistorySize + 1);

	ASSERT(parameters.pointCloudMapResolution > 0, "RegistrationFromStereo Error, Point Cloud Map resolution is not positive");
	pointCloudMap.SetResolution(parameters.pointCloudMapResolution);
	}

bool ReconstructionFromMotion::ComputeCameraMovement()
	{
	bundleHistory->AddImages(inLeftImage, inRightImage);

	FrameConstPtr filteredLeftImage = NULL;
	Execute(leftFilter, inLeftImage, filteredLeftImage);

	VisualPointFeatureVector2DConstPtr leftKeypointVector = NULL;
	VisualPointFeatureVector2DConstPtr leftFeatureVector = NULL;

	Execute(featuresExtractor, filteredLeftImage, leftKeypointVector);
	Execute(optionalFeaturesDescriptor, filteredLeftImage, leftKeypointVector, leftFeatureVector);
	bundleHistory->AddFeatures(*leftFeatureVector, LEFT_FEATURE_CATEGORY);

	bool success = false;
	for(int backwardStep = 1; backwardStep <= parameters.trackedHistorySize && !success; backwardStep++)
		{
		VisualPointFeatureVector2DConstPtr pastLeftFeatureVector = bundleHistory->GetFeatures(backwardStep, LEFT_FEATURE_CATEGORY);
		if (pastLeftFeatureVector == NULL)
			{
			break;
			}

		CorrespondenceMap2DConstPtr leftPastCorrespondenceMap = NULL;
		Execute(featuresMatcher, leftFeatureVector, pastLeftFeatureVector, leftPastCorrespondenceMap);

		Matrix3dConstPtr fundamentalMatrix = NULL;
		Execute(fundamentalMatrixComputer, leftPastCorrespondenceMap, fundamentalMatrix, success);

		if(success)
			{
			Pose3DConstPtr newPose = NULL;
			Execute(cameraTransformEstimator, fundamentalMatrix, leftPastCorrespondenceMap, newPose, success);
			if(success)
				{
				Copy(*newPose, poseToPreviousPose);
				}
			}

		}

	if(!success && !firstInput)
		{
		bundleHistory->RemoveEntry(0);
		}
	return success;
	}

PointCloudConstPtr ReconstructionFromMotion::ComputePointCloud()
	{
	VisualPointFeatureVector2DConstPtr leftFeatureVector = bundleHistory->GetFeatures(0, LEFT_FEATURE_CATEGORY);
	ASSERT(leftFeatureVector != NULL, "ReconstructionFromMotion error, ComputePointCloud() called before left feature vector was computed");

	FrameConstPtr filteredRightImage = NULL;
	Execute(leftFilter, inLeftImage, filteredRightImage);

	VisualPointFeatureVector2DConstPtr rightKeypointVector = NULL;
	VisualPointFeatureVector2DConstPtr rightFeatureVector = NULL;
	Execute(featuresExtractor, filteredRightImage, rightKeypointVector);
	Execute(optionalFeaturesDescriptor, filteredRightImage, rightKeypointVector, rightFeatureVector);

	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = NULL;
	Execute(featuresMatcher, leftFeatureVector, rightFeatureVector, leftRightCorrespondenceMap);

	PointCloudConstPtr pointCloud = NULL;
	Execute(reconstructor3D, *leftRightCorrespondenceMap, rightToLeftCameraPose, pointCloud);

	return pointCloud;
	}

void ReconstructionFromMotion::UpdateScene(PointCloudConstPtr inputCloud)
	{
	PointCloudWrapper::PointCloudConstPtr outputPointCloud = NULL;
	if (parameters.useAssemblerDfn)
		{
		if(firstInput)
			{
			Copy(zeroPose, outPose);
			}
		else
			{
			Pose3D newPose = Sum(outPose, poseToPreviousPose);
			Copy(newPose, outPose);
			}

		PointCloudConstPtr transformedCloud = NULL;
		Executors::Execute(cloudTransformer, *inputCloud, outPose, transformedCloud);
		Executors::Execute(cloudAssembler, *transformedCloud, outPose, parameters.searchRadius, outputPointCloud);
		}
	else 
		{
		if (firstInput)
			{
			pointCloudMap.AddPointCloud( inputCloud, EMPTY_FEATURE_VECTOR, &zeroPose);
			}
		else
			{
			pointCloudMap.AttachPointCloud( inputCloud, EMPTY_FEATURE_VECTOR, &poseToPreviousPose);
			}

		Copy( pointCloudMap.GetLatestPose(), outPose);
		outputPointCloud = pointCloudMap.GetScenePointCloudInOrigin(&outPose, parameters.searchRadius);
		}

	Copy(*outputPointCloud, outPointCloud);
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

	if (parameters.useAssemblerDfn)
		{
		cloudAssembler = static_cast<PointCloudAssemblyInterface*>( configurator.GetDfn("cloudAssembler") );
		cloudTransformer = static_cast<PointCloudTransformationInterface*>( configurator.GetDfn("cloudTransformer") );
		}
	}

}
}
}


/** @} */
