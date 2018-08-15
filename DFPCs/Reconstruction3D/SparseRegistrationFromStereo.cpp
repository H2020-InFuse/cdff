/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SparseRegistrationFromStereo.cpp
 * @date 05/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 *
 * Implementation of the SparseRegistrationFromStereo class.
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
#include "SparseRegistrationFromStereo.hpp"
#include "Errors/Assert.hpp"
#include <Visualizers/OpencvVisualizer.hpp>
#include <Visualizers/PclVisualizer.hpp>

#define DELETE_PREVIOUS(object) \
	{ \
	if (object != NULL) \
		{ \
		delete(object); \
		object = NULL; \
		} \
	} \

namespace CDFF
{
namespace DFPC
{
namespace Reconstruction3D
{

using namespace CDFF::DFN::WHICH-DFN(S)-IF-ANY?;
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
SparseRegistrationFromStereo::SparseRegistrationFromStereo()
	{
	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudMapResolution", parameters.pointCloudMapResolution, DEFAULT_PARAMETERS.pointCloudMapResolution);
	parametersHelper.AddParameter<float>("GeneralParameters", "SearchRadius", parameters.searchRadius, DEFAULT_PARAMETERS.searchRadius);

	leftImage = NewFrame();
	rightImage = NewFrame();
	filteredLeftImage = NULL;
	filteredRightImage = NULL;
	imagesCloud = NewPointCloud();
	imagesSparseCloud = NULL;
	sceneSparseCloud = NULL;
	imagesCloudKeypointsVector = NewVisualPointFeatureVector3D();
	sceneCloudKeypointsVector = NULL;
	cameraPoseInScene = NewPose3D();
	previousCameraPoseInScene = NewPose3D();
	emptyFeaturesVector = NewVisualPointFeatureVector3D();

	optionalLeftFilter = NULL;
	optionalRightFilter = NULL;
	reconstructor3D = NULL;
	featuresExtractor = NULL;
	cloudRegistrator = NULL;

	configurationFilePath = "";
	firstInput = true;
	}

SparseRegistrationFromStereo::~SparseRegistrationFromStereo()
	{
	if (optionalLeftFilter != NULL)
		{
		DELETE_PREVIOUS(filteredLeftImage);
		}
	if (optionalRightFilter != NULL)
		{
		DELETE_PREVIOUS(filteredRightImage);
		}
	delete(leftImage);
	delete(rightImage);
	delete(imagesCloud);
	DELETE_PREVIOUS(imagesSparseCloud);
	DELETE_PREVIOUS(sceneSparseCloud);
	delete(imagesCloudKeypointsVector);
	DELETE_PREVIOUS(sceneCloudKeypointsVector);
	delete(cameraPoseInScene);
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
void SparseRegistrationFromStereo::run()
	{
	DEBUG_PRINT_TO_LOG("Sparse Registration from stereo start", "");

	Copy(inLeftImage, *leftImage);
	Copy(inRightImage, *rightImage);

	ComputePointCloud();
	ComputeImagesCloudKeypoints();

	if (firstInput)
		{
		firstInput = false;

		cameraPoseInScene = NewPose3D();
		outSuccess = true;
		}
	else
		{
		DELETE_PREVIOUS(sceneCloudKeypointsVector);
		sceneCloudKeypointsVector = pointCloudMap.GetSceneFeaturesVector(previousCameraPoseInScene, parameters.searchRadius);

		DELETE_PREVIOUS(imagesSparseCloud);
		imagesSparseCloud = FromKeypointsToCloud(imagesCloudKeypointsVector);
		DELETE_PREVIOUS(sceneSparseCloud);
		sceneSparseCloud = FromKeypointsToCloud(sceneCloudKeypointsVector);
		outSuccess = RegisterImagesCloudOnScene();
		}

	if (outSuccess)
		{
		pointCloudMap.AddPointCloud(imagesCloud, imagesCloudKeypointsVector, cameraPoseInScene);
		PointCloudWrapper::PointCloudConstPtr outputPointCloud = pointCloudMap.GetScenePointCloudInOrigin(cameraPoseInScene, parameters.searchRadius);
		Copy(*outputPointCloud, outPointCloud); 
		DEBUG_SHOW_POINT_CLOUD(outputPointCloud);
		DELETE_PREVIOUS(outputPointCloud);

		Copy(*cameraPoseInScene, outPose);

		Copy(*cameraPoseInScene, *previousCameraPoseInScene);
		DEBUG_PRINT_TO_LOG("Pose ", ToString(outPose) );
		}
	}

void SparseRegistrationFromStereo::setup()
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

const SparseRegistrationFromStereo::SparseRegistrationFromStereoOptionsSet SparseRegistrationFromStereo::DEFAULT_PARAMETERS =
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
void SparseRegistrationFromStereo::ConfigureExtraParameters()
	{
	parametersHelper.ReadFile( configurator.GetExtraParametersConfigurationFilePath() );

	ASSERT(parameters.pointCloudMapResolution > 0, "SparseRegistrationFromStereo Error, Point Cloud Map resolution is not positive");
	pointCloudMap.SetResolution(parameters.pointCloudMapResolution);
	}

void SparseRegistrationFromStereo::AssignDfnsAlias()
	{
	optionalLeftFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) );
	optionalRightFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) );
	reconstructor3D = static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3D") );
	featuresExtractor = static_cast<FeaturesExtraction3DInterface*>( configurator.GetDfn("featuresExtractor3d") );
	cloudRegistrator = static_cast<Registration3DInterface*>( configurator.GetDfn("cloudRegistrator") );

	ASSERT(reconstructor3D != NULL, "DFPC Registration from stereo error: reconstructor3D DFN configured incorrectly");
	ASSERT(cloudRegistrator != NULL, "DFPC Registration from stereo error: featuresMatcher3d DFN configured incorrectly");

	if (optionalLeftFilter != NULL)
		{
		filteredLeftImage = NewFrame();
		}
	if (optionalRightFilter != NULL)
		{
		filteredRightImage = NewFrame();
		}
	}

/**
* The method filters the left and right images, and uses them for the computation of a point cloud.
*
**/
void SparseRegistrationFromStereo::ComputePointCloud()
	{
	FilterLeftImage();
	FilterRightImage();
	ComputeStereoPointCloud();
	}

void SparseRegistrationFromStereo::FilterLeftImage()
	{
	if (optionalLeftFilter != NULL)
		{
		optionalLeftFilter->imageInput(*leftImage);
		optionalLeftFilter->process();
		Copy(optionalLeftFilter->imageOutput(), *filteredLeftImage);
		DEBUG_PRINT_TO_LOG("Filtered Frame", "");
		DEBUG_SHOW_IMAGE(filteredLeftImage);
		}
	else
		{
		filteredLeftImage = leftImage;
		}
	}

void SparseRegistrationFromStereo::FilterRightImage()
	{
	if (optionalRightFilter != NULL)
		{
		optionalRightFilter->imageInput(*rightImage);
		optionalRightFilter->process();
		Copy(optionalRightFilter->imageOutput(), *filteredRightImage);
		DEBUG_PRINT_TO_LOG("Filtered Right Frame", "");
		DEBUG_SHOW_IMAGE(filteredRightImage);
		}
	else
		{
		filteredRightImage = rightImage;
		}
	}

void SparseRegistrationFromStereo::ComputeStereoPointCloud()
	{
	reconstructor3D->leftInput(*filteredLeftImage);
	reconstructor3D->rightInput(*filteredRightImage);
	reconstructor3D->process();
	Copy( reconstructor3D->pointcloudOutput(), *imagesCloud);
	DEBUG_PRINT_TO_LOG("Point Cloud", GetNumberOfPoints(*imagesCloud));
	DEBUG_SHOW_POINT_CLOUD(imagesCloud);
	}

void SparseRegistrationFromStereo::ComputeImagesCloudKeypoints()
	{
	featuresExtractor->pointcloudInput(*imagesCloud);
	featuresExtractor->process();
	Copy(featuresExtractor->featuresOutput(), *imagesCloudKeypointsVector);
	DEBUG_PRINT_TO_LOG("Extracted Point Cloud Features", GetNumberOfPoints(*imagesCloudKeypointsVector) );
	DEBUG_SHOW_3D_VISUAL_FEATURES(imagesCloud, imagesCloudKeypointsVector);
	}

PointCloudConstPtr SparseRegistrationFromStereo::FromKeypointsToCloud(VisualPointFeatureVector3DConstPtr keypointsVector)
	{
	PointCloudPtr pointCloud = NewPointCloud();
	for(int keypointIndex = 0; keypointIndex < GetNumberOfPoints(*keypointsVector); keypointIndex++)
		{
		AddPoint(*pointCloud, GetXCoordinate(*keypointsVector, keypointIndex), GetYCoordinate(*keypointsVector, keypointIndex), GetZCoordinate(*keypointsVector, keypointIndex));
		}
	return pointCloud;
	}

bool SparseRegistrationFromStereo::RegisterImagesCloudOnScene()
	{
	DEBUG_PRINT_TO_LOG("Starting Features Registration", "");
	cloudRegistrator->sourceCloudInput(*imagesSparseCloud);
	cloudRegistrator->sinkCloudInput(*sceneSparseCloud);
	cloudRegistrator->useGuessInput(false);
	cloudRegistrator->process();
	Copy(cloudRegistrator->transformOutput(), *cameraPoseInScene);
	bool registrationSuccess = cloudRegistrator->successOutput();
	DEBUG_PRINT_TO_LOG("Registration 3D Success", registrationSuccess );
	DEBUG_PRINT_TO_LOG("Transform", (registrationSuccess ? ToString(*cameraPoseInScene) : "") );
	return registrationSuccess;
	}

}
}
}


/** @} */
