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

	bundleHistory = new BundleHistory(2);

	#ifdef TESTING
	logFile.open("/InFuse/myLog.txt");
	logFile.close();
	#endif
	}

RegistrationFromStereo::~RegistrationFromStereo()
	{
	DeleteIfNotNull(optionalLeftFilter);
	DeleteIfNotNull(optionalRightFilter);
	DeleteIfNotNull(reconstructor3d);
	DeleteIfNotNull(featuresExtractor3d);
	DeleteIfNotNull(optionalFeaturesDescriptor3d);
	DeleteIfNotNull(featuresMatcher3d);
	DeleteIfNotNull(cloudAssembler);
	DeleteIfNotNull(cloudTransformer);
	DeleteIfNotNull(cloudFilter);
	DeleteIfNotNull(registrator3d);

	DeleteIfNotNull(bundleHistory);
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
	#ifdef TESTING
	logFile.open("/InFuse/myLog.txt", std::ios::app);
	#endif
	DEBUG_PRINT_TO_LOG("Registration from stereo start", "");

	bundleHistory->AddImages(inLeftImage, inRightImage);

	FrameConstPtr filteredLeftImage = NULL;
	FrameConstPtr filteredRightImage = NULL;
	optionalLeftFilter->Execute(inLeftImage, filteredLeftImage);
	optionalRightFilter->Execute(inRightImage, filteredRightImage);

	PointCloudConstPtr unfilteredImageCloud = NULL;
	reconstructor3d->Execute(filteredLeftImage, filteredRightImage, unfilteredImageCloud);

	PointCloudConstPtr imageCloud = NULL;
	cloudFilter->Execute(unfilteredImageCloud, imageCloud);

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
		PRINT_TO_LOG("Removing", "");
		bundleHistory->RemoveEntry(0);
		PRINT_TO_LOG("Removed", "");
		#ifdef TESTING
		logFile << std::endl;
		logFile.close();
		#endif
		return;
		}

	if (outSuccess)
		{
		UpdatePointCloud(imageCloud, featureVector);
		}

	#ifdef TESTING
	logFile << std::endl;
	logFile.close();
	#endif
	}

void RegistrationFromStereo::setup()
	{
	configurator.configure(configurationFilePath);
	ConfigureExtraParameters();
	InstantiateDFNExecutors();

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

void RegistrationFromStereo::InstantiateDFNExecutors()
	{
	optionalLeftFilter = new ImageFilteringExecutor( static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) ) );
	optionalRightFilter = new ImageFilteringExecutor( static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) ) );
	reconstructor3d = new StereoReconstructionExecutor( static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3D") ) );
	featuresExtractor3d = new FeaturesExtraction3DExecutor( static_cast<FeaturesExtraction3DInterface*>( configurator.GetDfn("featuresExtractor3d") ) );
	optionalFeaturesDescriptor3d = new FeaturesDescription3DExecutor( static_cast<FeaturesDescription3DInterface*>( configurator.GetDfn("featuresDescriptor3d", true) ) );
	featuresMatcher3d = new FeaturesMatching3DExecutor( static_cast<FeaturesMatching3DInterface*>( configurator.GetDfn("featuresMatcher3d") ) );
	cloudFilter = new PointCloudFilteringExecutor( static_cast<PointCloudFilteringInterface*>( configurator.GetDfn("cloudFilter", true) ) );
	if (parameters.useRegistratorDfn)
		{
		registrator3d = new Registration3DExecutor( static_cast<Registration3DInterface*>( configurator.GetDfn("registrator3d") ) );
		}
	if (parameters.useAssemblerDfn)
		{
		cloudAssembler = new PointCloudAssemblyExecutor( static_cast<PointCloudAssemblyInterface*>( configurator.GetDfn("cloudAssembler") ) );
		}
	if (parameters.useRegistratorDfn || parameters.useAssemblerDfn)
		{
		cloudTransformer = new PointCloudTransformExecutor( static_cast<PointCloudTransformInterface*>( configurator.GetDfn("cloudTransformer") ) );
		}
	}

#ifdef TESTING
void RegistrationFromStereo::WriteOutputToLogFile()
	{
	logFile << GetNumberOfPoints(outPointCloud) << " ";
	logFile << GetXPosition(outPose) << " ";
	logFile << GetYPosition(outPose) << " ";
	logFile << GetZPosition(outPose) << " ";
	logFile << GetXOrientation(outPose) << " ";
	logFile << GetYOrientation(outPose) << " ";
	logFile << GetZOrientation(outPose) << " ";
	logFile << GetWOrientation(outPose) << " ";

	if ( GetNumberOfPoints(outPointCloud) > 0)
		{
		float minMax[6] = { GetXCoordinate(outPointCloud, 0), GetXCoordinate(outPointCloud, 0), GetYCoordinate(outPointCloud, 0), 
			GetYCoordinate(outPointCloud, 0), GetZCoordinate(outPointCloud, 0), GetZCoordinate(outPointCloud, 0)};
		int numberOfPoints = GetNumberOfPoints(outPointCloud);
		for (int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
			{
			bool change[6];
			change[0] = minMax[0] < GetXCoordinate(outPointCloud, pointIndex);
			change[1] = minMax[1] > GetXCoordinate(outPointCloud, pointIndex);
			change[2] = minMax[2] < GetYCoordinate(outPointCloud, pointIndex);
			change[3] = minMax[3] > GetYCoordinate(outPointCloud, pointIndex);
			change[4] = minMax[4] < GetZCoordinate(outPointCloud, pointIndex);
			change[5] = minMax[5] > GetZCoordinate(outPointCloud, pointIndex);
			minMax[0] = change[0] ? GetXCoordinate(outPointCloud, pointIndex) : minMax[0];
			minMax[1] = change[1] ? GetXCoordinate(outPointCloud, pointIndex) : minMax[1];
			minMax[2] = change[2] ? GetYCoordinate(outPointCloud, pointIndex) : minMax[2];
			minMax[3] = change[3] ? GetYCoordinate(outPointCloud, pointIndex) : minMax[3];
			minMax[4] = change[4] ? GetZCoordinate(outPointCloud, pointIndex) : minMax[4];
			minMax[5] = change[5] ? GetZCoordinate(outPointCloud, pointIndex) : minMax[5];
			}
		logFile << (minMax[0] - minMax[1]) << " ";
		logFile << (minMax[2] - minMax[3]) << " ";
		logFile << (minMax[4] - minMax[5]) << " ";
		}
	else	
		{
		logFile << 0 << " ";
		logFile << 0 << " ";
		logFile << 0 << " ";
		}
	}
#endif

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
		featuresMatcher3d->Execute( featureVector, bundleHistory->GetFeatures3d(1), poseToPreviousPoseClose, outSuccess);

		if (outSuccess)
			{
			if (parameters.useRegistratorDfn)
				{
				PointCloudConstPtr closerCloud = NULL;
				cloudTransformer->Execute(imageCloud, poseToPreviousPoseClose, closerCloud);

				Pose3DConstPtr poseToPreviousPoseCloser = NULL;
				registrator3d->Execute( closerCloud, bundleHistory->GetPointCloud(1), poseToPreviousPoseCloser, outSuccess);

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
		#ifdef TESTING
		logFile << outSuccess << " ";
		logFile << GetXPosition(*poseToPreviousPose) << " ";
		logFile << GetYPosition(*poseToPreviousPose) << " ";
		logFile << GetZPosition(*poseToPreviousPose) << " ";
		logFile << GetXOrientation(*poseToPreviousPose) << " ";
		logFile << GetYOrientation(*poseToPreviousPose) << " ";
		logFile << GetZOrientation(*poseToPreviousPose) << " ";
		logFile << GetWOrientation(*poseToPreviousPose) << " ";
		#endif
		}
	}

void RegistrationFromStereo::UpdatePointCloud(PointCloudConstPtr imageCloud, VisualPointFeatureVector3DConstPtr featureVector)
	{
	PointCloudWrapper::PointCloudConstPtr outputPointCloud = NULL;
	if (parameters.useAssemblerDfn)
		{
		PointCloudConstPtr transformedImageCloud = NULL;
		cloudTransformer->Execute(*imageCloud, outPose, transformedImageCloud);
		cloudAssembler->Execute(*transformedImageCloud, outPose, parameters.searchRadius, outputPointCloud);
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

	#ifdef TESTING
	WriteOutputToLogFile();
	#endif

	DEBUG_SHOW_POINT_CLOUD(outputPointCloud);
	if (!parameters.useAssemblerDfn)
		{
		DeleteIfNotNull(outputPointCloud);
		}
	}

void RegistrationFromStereo::ComputeVisualFeatures(PointCloudConstPtr inputCloud, VisualPointFeatureVector3DConstPtr& outputFeatures)
	{
	ASSERT(outputFeatures == NULL, "RegistrationFromStereo error! ComputeVisualFeatures was called while outputFeatures is not NULL. OutputFeatures will be overwritten, look for memory leaks.");
	VisualPointFeatureVector3DConstPtr keypointVector = NULL;
	featuresExtractor3d->Execute(inputCloud, keypointVector);
	DEBUG_SHOW_3D_VISUAL_FEATURES(inputCloud, keypointVector);

	optionalFeaturesDescriptor3d->Execute(inputCloud, keypointVector, outputFeatures);
	DEBUG_PRINT_TO_LOG("Described Features:", GetNumberOfPoints(*outputFeatures));

	#ifdef TESTING
	logFile << GetNumberOfPoints(*inputCloud) << " ";
	logFile << GetNumberOfPoints(*keypointVector) << " ";
	logFile << GetNumberOfPoints(*outputFeatures) << " ";
	#endif
	}

}
}
}


/** @} */
