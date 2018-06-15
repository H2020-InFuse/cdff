/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DenseRegistrationFromStereo.cpp
 * @date 31/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 *
 * Implementation of the DenseRegistrationFromStereo class.
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
#include "DenseRegistrationFromStereo.hpp"
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
using namespace FrameWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector3DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
DenseRegistrationFromStereo::DenseRegistrationFromStereo()
	{
	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudMapResolution", parameters.pointCloudMapResolution, DEFAULT_PARAMETERS.pointCloudMapResolution);
	parametersHelper.AddParameter<float>("GeneralParameters", "SearchRadius", parameters.searchRadius, DEFAULT_PARAMETERS.searchRadius);

	filteredLeftImage = NULL;
	filteredRightImage = NULL;
	imagesCloud = NULL;
	cameraPoseInScene = NULL;
	previousCameraPoseInScene = NewPose3D();
	emptyFeaturesVector = NewVisualPointFeatureVector3D();

	optionalLeftFilter = NULL;
	optionalRightFilter = NULL;
	reconstructor3D = NULL;
	cloudRegistrator = NULL;

	configurationFilePath = "";
	firstInput = true;
	}

DenseRegistrationFromStereo::~DenseRegistrationFromStereo()
	{
	if (optionalLeftFilter != NULL)
		{
		DELETE_PREVIOUS(filteredLeftImage);
		}
	if (optionalRightFilter != NULL)
		{
		DELETE_PREVIOUS(filteredRightImage);
		}
	DELETE_PREVIOUS(imagesCloud);
	DELETE_PREVIOUS(cameraPoseInScene);
	DELETE_PREVIOUS(previousCameraPoseInScene);
	DELETE_PREVIOUS(emptyFeaturesVector);
	}

/**
* The process method is split into three steps
* (i) computation of the point cloud from the stereo pair;
* (ii) computation of the camera pose by 3d matching of the point cloud with the a partial scene of the original map ceneters at the camera previous pose;
* (iii) the point cloud rover map is updated with the newly computed point cloud.
*
**/
void DenseRegistrationFromStereo::run()
	{
	DEBUG_PRINT_TO_LOG("Dense Registration from stereo start", "");

	leftImage = inLeftImage;
	rightImage = inRightImage;

	ComputePointCloud();

	if (firstInput)
		{
		firstInput = false;

		cameraPoseInScene = NewPose3D();
		outSuccess = true;
		}
	else
		{
		sceneCloud = pointCloudMap.GetScenePointCloudInOrigin(previousCameraPoseInScene, parameters.searchRadius);
		outSuccess = RegisterPointCloudOnScene();
		}

	if (outSuccess)
		{
		pointCloudMap.AddPointCloud(imagesCloud, emptyFeaturesVector, cameraPoseInScene);
		outPointCloud = pointCloudMap.GetScenePointCloudInOrigin(cameraPoseInScene, parameters.searchRadius);
		DEBUG_SHOW_POINT_CLOUD(outPointCloud);

		Pose3DPtr newOutPose = NewPose3D();
		Copy(*cameraPoseInScene, *newOutPose);
		outPose = newOutPose;

		Copy(*cameraPoseInScene, *previousCameraPoseInScene);
		DEBUG_PRINT_TO_LOG("Pose ", ToString(*outPose) );
		}
	else
		{
		outPointCloud = NULL;
		outPose = NULL;
		}
	}

void DenseRegistrationFromStereo::setup()
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

const DenseRegistrationFromStereo::DenseRegistrationFromStereoOptionsSet DenseRegistrationFromStereo::DEFAULT_PARAMETERS =
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
void DenseRegistrationFromStereo::ConfigureExtraParameters()
	{
	parametersHelper.ReadFile( configurator.GetExtraParametersConfigurationFilePath() );

	ASSERT(parameters.pointCloudMapResolution > 0, "DenseRegistrationFromStereo Error, Point Cloud Map resolution is not positive");
	pointCloudMap.SetResolution(parameters.pointCloudMapResolution);
	}

void DenseRegistrationFromStereo::AssignDfnsAlias()
	{
	optionalLeftFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) );
	optionalRightFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) );
	reconstructor3D = static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3D") );
	cloudRegistrator = static_cast<Registration3DInterface*>( configurator.GetDfn("cloudRegistrator") );

	ASSERT(reconstructor3D != NULL, "DFPC Registration from stereo error: reconstructor3D DFN configured incorrectly");
	ASSERT(cloudRegistrator != NULL, "DFPC Registration from stereo error: featuresMatcher3d DFN configured incorrectly");
	}

/**
* The method filters the left and right images, and uses them for the computation of a point cloud.
*
**/
void DenseRegistrationFromStereo::ComputePointCloud()
	{
	FilterLeftImage();
	FilterRightImage();
	ComputeStereoPointCloud();
	}

void DenseRegistrationFromStereo::FilterLeftImage()
	{
	if (optionalLeftFilter != NULL)
		{
		optionalLeftFilter->imageInput(*leftImage);
		optionalLeftFilter->process();
		DELETE_PREVIOUS(filteredLeftImage);
		FramePtr newFrame = NewFrame();
		Copy(optionalLeftFilter->imageOutput(), *newFrame);
		filteredLeftImage = newFrame;
		DEBUG_PRINT_TO_LOG("Filtered Frame", "");
		DEBUG_SHOW_IMAGE(filteredLeftImage);
		}
	else
		{
		filteredLeftImage = leftImage;
		}
	}

void DenseRegistrationFromStereo::FilterRightImage()
	{
	if (optionalRightFilter != NULL)
		{
		optionalRightFilter->imageInput(*rightImage);
		optionalRightFilter->process();
		DELETE_PREVIOUS(filteredRightImage);
		FramePtr newFrame = NewFrame();
		Copy(optionalRightFilter->imageOutput(), *newFrame);
		filteredRightImage = newFrame;
		DEBUG_PRINT_TO_LOG("Filtered Right Frame", "");
		DEBUG_SHOW_IMAGE(filteredRightImage);
		}
	else
		{
		filteredRightImage = rightImage;
		}
	}

void DenseRegistrationFromStereo::ComputeStereoPointCloud()
	{
	reconstructor3D->leftInput(*filteredLeftImage);
	reconstructor3D->rightInput(*filteredRightImage);
	reconstructor3D->process();
	DELETE_PREVIOUS(imagesCloud);
	const PointCloud& tmp = reconstructor3D->pointcloudOutput();
	imagesCloud = &tmp;
	DEBUG_PRINT_TO_LOG("Point Cloud", GetNumberOfPoints(*imagesCloud));
	DEBUG_SHOW_POINT_CLOUD(imagesCloud);
	}

bool DenseRegistrationFromStereo::RegisterPointCloudOnScene()
	{
	DEBUG_PRINT_TO_LOG("Starting Registration", "");
	cloudRegistrator->sourceCloudInput(*imagesCloud);
	cloudRegistrator->sinkCloudInput(*sceneCloud);
	cloudRegistrator->useGuessInput(false);
	cloudRegistrator->process();
	DELETE_PREVIOUS(cameraPoseInScene);
	Pose3DPtr newPose = NewPose3D();
	Copy(cloudRegistrator->transformOutput(), *newPose);
	cameraPoseInScene = newPose;
	bool registrationSuccess = cloudRegistrator->successOutput();
	DEBUG_PRINT_TO_LOG("Registration 3D Success", registrationSuccess );
	DEBUG_PRINT_TO_LOG("Transform", (registrationSuccess ? ToString(*cameraPoseInScene) : "") );
	return registrationSuccess;
	}

}


/** @} */
