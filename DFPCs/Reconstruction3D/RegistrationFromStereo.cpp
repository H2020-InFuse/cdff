/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file RegistrationFromStereo.cpp
 * @date 18/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the RegistrationFromStereo class.
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
#include "RegistrationFromStereo.hpp"
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
using namespace VisualPointFeatureVector3DWrapper;
using namespace FrameWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
RegistrationFromStereo::RegistrationFromStereo()
	{
	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudMapResolution", parameters.pointCloudMapResolution, DEFAULT_PARAMETERS.pointCloudMapResolution);
	parametersHelper.AddParameter<float>("GeneralParameters", "SearchRadius", parameters.searchRadius, DEFAULT_PARAMETERS.searchRadius);

	leftImage = NewFrame();
	rightImage = NewFrame();
	filteredLeftImage = NULL;
	filteredRightImage = NULL;
	pointCloud = NewPointCloud();
	pointCloudKeypointsVector = NewVisualPointFeatureVector3D();
	pointCloudFeaturesVector = NULL;
	sceneFeaturesVector = NULL;
	cameraPoseInScene = NewPose3D();
	previousCameraPoseInScene = NewPose3D();

	optionalLeftFilter = NULL;
	optionalRightFilter = NULL;
	reconstructor3D = NULL;
	featuresExtractor3d = NULL;
	optionalFeaturesDescriptor3d = NULL;
	featuresMatcher3d = NULL;

	configurationFilePath = "";
	firstInput = true;
	}

RegistrationFromStereo::~RegistrationFromStereo()
	{
	if (optionalLeftFilter != NULL)
		{
		DELETE_PREVIOUS(filteredLeftImage);
		}
	if (optionalRightFilter != NULL)
		{
		DELETE_PREVIOUS(filteredRightImage);
		}
	if (optionalFeaturesDescriptor3d != NULL)
		{
		DELETE_PREVIOUS(pointCloudFeaturesVector);
		}
	delete(leftImage);
	delete(rightImage);
	delete(pointCloud);
	delete(pointCloudKeypointsVector);
	DELETE_PREVIOUS(sceneFeaturesVector);
	delete(cameraPoseInScene);
	delete(previousCameraPoseInScene);
	}

/**
* The process method is split into three steps 
* (i) computation of the point cloud from the stereo pair;
* (ii) computation of the camera pose by 3d matching of the point cloud with the a partial scene of the original map ceneters at the camera previous pose;
* (iii) the point cloud rover map is updated with the newly computed point cloud.
*
**/
void RegistrationFromStereo::run() 
	{
	DEBUG_PRINT_TO_LOG("Registration from stereo start", "");

	Copy(inLeftImage, *leftImage);
	Copy(inRightImage, *rightImage);

	ComputePointCloud();

	if (firstInput)
		{
		firstInput = false;

		ExtractPointCloudFeatures();
		DescribePointCloudFeatures();
		cameraPoseInScene = NewPose3D();	
		outSuccess = true;
		}
	else
		{
		outSuccess = ComputeCameraMovement();
		}

	if (outSuccess)
		{
		pointCloudMap.AddPointCloud(pointCloud, pointCloudFeaturesVector, cameraPoseInScene);
		PointCloudConstPtr outputPointCloud = pointCloudMap.GetScenePointCloud(cameraPoseInScene, parameters.searchRadius);
		Copy(*outputPointCloud, outPointCloud);
		DEBUG_SHOW_POINT_CLOUD(outputPointCloud);

		Copy(*cameraPoseInScene, outPose);

		Copy(*cameraPoseInScene, *previousCameraPoseInScene);
		DEBUG_PRINT_TO_LOG("Pose ", ToString(outPose) );
		}
	}

void RegistrationFromStereo::setup()
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

const RegistrationFromStereo::RegistrationFromStereoOptionsSet RegistrationFromStereo::DEFAULT_PARAMETERS = 
	{
	.searchRadius = 20,
	.pointCloudMapResolution = 1e-2
	};

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void RegistrationFromStereo::ConfigureExtraParameters()
	{
	parametersHelper.ReadFile( configurator.GetExtraParametersConfigurationFilePath() );

	ASSERT(parameters.pointCloudMapResolution > 0, "RegistrationFromStereo Error, Point Cloud Map resolution is not positive");
	pointCloudMap.SetResolution(parameters.pointCloudMapResolution);
	}

void RegistrationFromStereo::AssignDfnsAlias()
	{
	optionalLeftFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) );
	optionalRightFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) );
	reconstructor3D = static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3D") );
	featuresExtractor3d = static_cast<FeaturesExtraction3DInterface*>( configurator.GetDfn("featuresExtractor3d") );
	optionalFeaturesDescriptor3d = static_cast<FeaturesDescription3DInterface*>( configurator.GetDfn("featuresDescriptor3d", true) );
	featuresMatcher3d = static_cast<FeaturesMatching3DInterface*>( configurator.GetDfn("featuresMatcher3d") );

	ASSERT(reconstructor3D != NULL, "DFPC Registration from stereo error: reconstructor3D DFN configured incorrectly");
	ASSERT(featuresMatcher3d != NULL, "DFPC Registration from stereo error: featuresMatcher3d DFN configured incorrectly");
	ASSERT(featuresExtractor3d != NULL, "DFPC Registration from stereo error: featuresExtractor3d DFN configured incorrectly");

	if (optionalLeftFilter != NULL)
		{
		filteredLeftImage = NewFrame();
		}
	if (optionalRightFilter != NULL)
		{
		filteredRightImage = NewFrame();
		}
	if (optionalFeaturesDescriptor3d != NULL)
		{
		pointCloudFeaturesVector = NewVisualPointFeatureVector3D();
		}
	}

/**
* The method filters the left and right images, and uses them for the computation of a point cloud.
*
**/
void RegistrationFromStereo::ComputePointCloud()
	{
	FilterLeftImage();	
	FilterRightImage();
	ComputeStereoPointCloud();
	}


/**
* The ComputeCameraMovement Method performs the following operation
*
* (i) Retrieve the scene features previously stored
* (ii) Determine the position of the currently detected point cloud by matching of 3d features
*
**/
bool RegistrationFromStereo::ComputeCameraMovement()
	{
	DELETE_PREVIOUS(sceneFeaturesVector);
	sceneFeaturesVector = pointCloudMap.GetSceneFeaturesVector(previousCameraPoseInScene, parameters.searchRadius);

	ExtractPointCloudFeatures();
	DescribePointCloudFeatures();
	bool success = MatchPointCloudWithSceneFeatures();	
	return success;
	}

void RegistrationFromStereo::FilterLeftImage()
	{
	if (optionalLeftFilter != NULL)
		{
		optionalLeftFilter->imageInput(*leftImage);
		optionalLeftFilter->process();
		Copy( optionalLeftFilter->imageOutput(), *filteredLeftImage);
		DEBUG_PRINT_TO_LOG("Filtered Frame", "");
		DEBUG_SHOW_IMAGE(filteredLeftImage);
		}
	else
		{
		filteredLeftImage = leftImage;
		}
	}

void RegistrationFromStereo::FilterRightImage()
	{
	if (optionalRightFilter != NULL)
		{
		optionalRightFilter->imageInput(*rightImage);
		optionalRightFilter->process();
		Copy( optionalRightFilter->imageOutput(), *filteredRightImage);
		DEBUG_PRINT_TO_LOG("Filtered Right Frame", "");
		DEBUG_SHOW_IMAGE(filteredRightImage);
		}
	else
		{
		filteredRightImage = rightImage;
		}	
	}

void RegistrationFromStereo::ComputeStereoPointCloud()
	{
	reconstructor3D->leftInput(*filteredLeftImage);
	reconstructor3D->rightInput(*filteredRightImage);
	reconstructor3D->process();
	Copy( reconstructor3D->pointcloudOutput(), *pointCloud);
	DEBUG_PRINT_TO_LOG("Point Cloud", GetNumberOfPoints(*pointCloud));
	DEBUG_SHOW_POINT_CLOUD(pointCloud);
	}

void RegistrationFromStereo::ExtractPointCloudFeatures()
	{
	featuresExtractor3d->pointcloudInput(*pointCloud);
	featuresExtractor3d->process();
	Copy( featuresExtractor3d->featuresOutput(), *pointCloudKeypointsVector);
	DEBUG_PRINT_TO_LOG("Extracted Point Cloud Features", GetNumberOfPoints(*pointCloudKeypointsVector) );
	DEBUG_SHOW_3D_VISUAL_FEATURES(pointCloud, pointCloudKeypointsVector);
	}

void RegistrationFromStereo::DescribePointCloudFeatures()
	{
	if (optionalFeaturesDescriptor3d != NULL)
		{
		optionalFeaturesDescriptor3d->pointcloudInput(*pointCloud);
		optionalFeaturesDescriptor3d->featuresInput(*pointCloudKeypointsVector);
		optionalFeaturesDescriptor3d->process();
		Copy( optionalFeaturesDescriptor3d->featuresOutput(), *pointCloudFeaturesVector);
		DEBUG_PRINT_TO_LOG("Described Point Cloud Features", GetNumberOfPoints(*pointCloudFeaturesVector) );
		}
	else
		{
		pointCloudFeaturesVector = pointCloudKeypointsVector;
		}
	}

bool RegistrationFromStereo::MatchPointCloudWithSceneFeatures()
	{
	featuresMatcher3d->sourceFeaturesInput(*pointCloudFeaturesVector);
	featuresMatcher3d->sinkFeaturesInput(*sceneFeaturesVector);
	featuresMatcher3d->process();
	Copy( featuresMatcher3d->transformOutput(), *cameraPoseInScene);
	bool matching3dSuccess = featuresMatcher3d->successOutput();	
	DEBUG_PRINT_TO_LOG("Matching 3d Success", matching3dSuccess );
	return matching3dSuccess;
	}

}
}
}


/** @} */
