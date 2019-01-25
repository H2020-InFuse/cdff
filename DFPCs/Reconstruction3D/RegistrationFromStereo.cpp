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
#include <Errors/Assert.hpp>
#include <Errors/AssertOnTest.hpp>

#include <Executors/ImageFiltering/ImageFilteringExecutor.hpp>
#include <Executors/StereoReconstruction/StereoReconstructionExecutor.hpp>
#include <Executors/FeaturesExtraction3D/FeaturesExtraction3DExecutor.hpp>
#include <Executors/FeaturesDescription3D/FeaturesDescription3DExecutor.hpp>
#include <Executors/FeaturesMatching3D/FeaturesMatching3DExecutor.hpp>
#include <Executors/PointCloudAssembly/PointCloudAssemblyExecutor.hpp>
#include <Executors/PointCloudTransformation/PointCloudTransformationExecutor.hpp>
#include <Executors/PointCloudFiltering/PointCloudFilteringExecutor.hpp>
#include <Executors/Registration3D/Registration3DExecutor.hpp>

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
	parameters = DEFAULT_PARAMETERS;

	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudMapResolution", parameters.pointCloudMapResolution, DEFAULT_PARAMETERS.pointCloudMapResolution);
	parametersHelper.AddParameter<float>("GeneralParameters", "SearchRadius", parameters.searchRadius, DEFAULT_PARAMETERS.searchRadius);
	parametersHelper.AddParameter<bool>("GeneralParameters", "MatchToReconstructedCloud", parameters.matchToReconstructedCloud, DEFAULT_PARAMETERS.matchToReconstructedCloud);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseAssemblerDfn", parameters.useAssemblerDfn, DEFAULT_PARAMETERS.useAssemblerDfn);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseRegistratorDfn", parameters.useRegistratorDfn, DEFAULT_PARAMETERS.useRegistratorDfn);

	configurationFilePath = "";
	firstInput = true;

	optionalLeftFilter = NULL;
	optionalRightFilter = NULL;
	reconstructor3d = NULL;
	featuresExtractor3d = NULL;
	optionalFeaturesDescriptor3d = NULL;
	featuresMatcher3d = NULL;
	cloudAssembler = NULL;
	cloudTransformer = NULL;
	cloudFilter = NULL;
	reconstructor3d = NULL;
	registrator3d = NULL;

	bundleHistory = new BundleHistory(2);
	}

RegistrationFromStereo::~RegistrationFromStereo()
	{
	DeleteIfNotNull(bundleHistory);
	}

void RegistrationFromStereo::run() 
	{
	DEBUG_PRINT_TO_LOG("Registration from stereo start", "");

	bundleHistory->AddImages(inLeftImage, inRightImage);

	FrameConstPtr filteredLeftImage = NULL;
	FrameConstPtr filteredRightImage = NULL;
	Executors::Execute(optionalLeftFilter, inLeftImage, filteredLeftImage);
	Executors::Execute(optionalRightFilter, inRightImage, filteredRightImage);

	PointCloudConstPtr unfilteredImageCloud = NULL;
	Executors::Execute(reconstructor3d, filteredLeftImage, filteredRightImage, unfilteredImageCloud);

	PointCloudConstPtr imageCloud = NULL;
	Executors::Execute(cloudFilter, unfilteredImageCloud, imageCloud);

	VisualPointFeatureVector3DConstPtr featureVector = NULL;
	ComputeVisualFeatures(imageCloud, featureVector);
	PRINT_TO_LOG("features", GetNumberOfPoints(*featureVector));

	if (!parameters.matchToReconstructedCloud)
		{
		bundleHistory->AddFeatures3d(*featureVector);
		}

	UpdatePose(imageCloud, featureVector);

	if (!outSuccess)
		{
		bundleHistory->RemoveEntry(0);
		return;
		}

	if (outSuccess)
		{
		UpdatePointCloud(imageCloud, featureVector);
		}
	}

void RegistrationFromStereo::setup()
	{
	configurator.configure(configurationFilePath);
	ConfigureExtraParameters();
	InstantiateDFNs();

	pointCloudMap.SetResolution(parameters.pointCloudMapResolution);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */

const RegistrationFromStereo::RegistrationFromStereoOptionsSet RegistrationFromStereo::DEFAULT_PARAMETERS = 
	{
	/*.searchRadius =*/ 20,
	/*.pointCloudMapResolution =*/ 1e-2,
	/*.matchToReconstructedCloud =*/ false,
	/*.useAssemblerDfn=*/ false,
	/*.useRegistratorDfn=*/ false
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
	}

void RegistrationFromStereo::InstantiateDFNs()
	{
	optionalLeftFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) );
	optionalRightFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) );
	reconstructor3d = static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3D") );
	featuresExtractor3d = static_cast<FeaturesExtraction3DInterface*>( configurator.GetDfn("featuresExtractor3d") );
	optionalFeaturesDescriptor3d = static_cast<FeaturesDescription3DInterface*>( configurator.GetDfn("featuresDescriptor3d", true) );
	featuresMatcher3d = static_cast<FeaturesMatching3DInterface*>( configurator.GetDfn("featuresMatcher3d") );
	cloudFilter = static_cast<PointCloudFilteringInterface*>( configurator.GetDfn("cloudFilter", true) );
	if (parameters.useRegistratorDfn)
		{
		registrator3d = static_cast<Registration3DInterface*>( configurator.GetDfn("registrator3d") );
		}
	if (parameters.useAssemblerDfn)
		{
		cloudAssembler = static_cast<PointCloudAssemblyInterface*>( configurator.GetDfn("cloudAssembler") );
		}
	if (parameters.useRegistratorDfn || parameters.useAssemblerDfn)
		{
		cloudTransformer = static_cast<PointCloudTransformationInterface*>( configurator.GetDfn("cloudTransformer") );
		}
	}

void RegistrationFromStereo::UpdatePose(PointCloudConstPtr imageCloud, VisualPointFeatureVector3DConstPtr featureVector)
	{
	if (firstInput)
		{
		firstInput = false;
		outSuccess = true;

		Pose3D zeroPose;
		SetPosition(zeroPose, 0, 0, 0);
		SetOrientation(zeroPose, 0, 0, 0, 1);
		if (!parameters.useAssemblerDfn)
			{
			pointCloudMap.AddPointCloud( imageCloud, featureVector, &zeroPose);
			}
		Copy(zeroPose, outPose);
		}
	else
		{
		Pose3DPtr poseToPreviousPose = NewPose3D();

		Pose3DConstPtr poseToPreviousPoseClose = NULL;
		Executors::Execute(featuresMatcher3d, featureVector, bundleHistory->GetFeatures3d(1), poseToPreviousPoseClose, outSuccess);

		if (outSuccess)
			{
			if (parameters.useRegistratorDfn)
				{
				PointCloudConstPtr closerCloud = NULL;
				Executors::Execute(cloudTransformer, imageCloud, poseToPreviousPoseClose, closerCloud);

				Pose3DConstPtr poseToPreviousPoseCloser = NULL;
				Executors::Execute(registrator3d, closerCloud, bundleHistory->GetPointCloud(1), poseToPreviousPoseCloser, outSuccess);

				(*poseToPreviousPose) = Sum(*poseToPreviousPoseClose, *poseToPreviousPoseCloser);
				}
			else
				{
				Copy(*poseToPreviousPoseClose, *poseToPreviousPose);
				}
			}
		if (outSuccess)
			{
			if (parameters.useAssemblerDfn && parameters.matchToReconstructedCloud)
				{
				Copy(*poseToPreviousPose, outPose);
				}
			else if (parameters.useAssemblerDfn && !parameters.matchToReconstructedCloud)
				{
				Pose3D newPose = Sum(outPose, *poseToPreviousPose);
				Copy(newPose, outPose);
				}
			else if (!parameters.useAssemblerDfn && parameters.matchToReconstructedCloud)
				{
				pointCloudMap.AddPointCloud( imageCloud, featureVector, poseToPreviousPose);
				Copy(*poseToPreviousPose, outPose);
				}
			else
				{
				pointCloudMap.AttachPointCloud( imageCloud, featureVector, poseToPreviousPose);
				Copy( pointCloudMap.GetLatestPose(), outPose);
				}
			}
		delete(poseToPreviousPose);
		}
	}

void RegistrationFromStereo::UpdatePointCloud(PointCloudConstPtr imageCloud, VisualPointFeatureVector3DConstPtr featureVector)
	{
	PointCloudWrapper::PointCloudConstPtr outputPointCloud = NULL;
	if (parameters.useAssemblerDfn)
		{
		PointCloudConstPtr transformedImageCloud = NULL;
		Executors::Execute(cloudTransformer, *imageCloud, outPose, transformedImageCloud);
		Executors::Execute(cloudAssembler, *transformedImageCloud, outPose, parameters.searchRadius, outputPointCloud);
		if (parameters.matchToReconstructedCloud)
			{
			VisualPointFeatureVector3DConstPtr reconstructedFeatureVector = NULL;
			ComputeVisualFeatures(outputPointCloud, reconstructedFeatureVector);

			bundleHistory->AddFeatures3d(*reconstructedFeatureVector);
			}
		}
	else
		{
		if (parameters.matchToReconstructedCloud)
			{
			VisualPointFeatureVector3DConstPtr fullCloudFeatureVector = pointCloudMap.GetSceneFeaturesVector(&outPose, parameters.searchRadius);
			bundleHistory->AddFeatures3d(*fullCloudFeatureVector);
			DeleteIfNotNull(fullCloudFeatureVector);
			}
		outputPointCloud = pointCloudMap.GetScenePointCloudInOrigin(&outPose, parameters.searchRadius);
		}

	Copy(*outputPointCloud, outPointCloud);
	if (parameters.matchToReconstructedCloud)
		{
		bundleHistory->AddPointCloud(*outputPointCloud);
		}

	DEBUG_PRINT_TO_LOG("pose", ToString(outPose));
	DEBUG_PRINT_TO_LOG("points", GetNumberOfPoints(outPointCloud));

	if (!parameters.useAssemblerDfn)
		{
		DeleteIfNotNull(outputPointCloud);
		}
	}

void RegistrationFromStereo::ComputeVisualFeatures(PointCloudConstPtr inputCloud, VisualPointFeatureVector3DConstPtr& outputFeatures)
	{
	ASSERT(outputFeatures == NULL, "RegistrationFromStereo error! ComputeVisualFeatures was called while outputFeatures is not NULL. OutputFeatures will be overwritten, look for memory leaks.");
	VisualPointFeatureVector3DConstPtr keypointVector = NULL;
	Executors::Execute(featuresExtractor3d, inputCloud, keypointVector);

	Executors::Execute(optionalFeaturesDescriptor3d, inputCloud, keypointVector, outputFeatures);
	DEBUG_PRINT_TO_LOG("Described Features:", GetNumberOfPoints(*outputFeatures));
	}

}
}
}


/** @} */
