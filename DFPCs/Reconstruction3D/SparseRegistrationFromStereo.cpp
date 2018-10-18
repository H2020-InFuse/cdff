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
SparseRegistrationFromStereo::SparseRegistrationFromStereo()
	{
	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudMapResolution", parameters.pointCloudMapResolution, DEFAULT_PARAMETERS.pointCloudMapResolution);
	parametersHelper.AddParameter<float>("GeneralParameters", "SearchRadius", parameters.searchRadius, DEFAULT_PARAMETERS.searchRadius);
	parametersHelper.AddParameter<bool>("GeneralParameters", "MatchToReconstructedCloud", parameters.matchToReconstructedCloud, DEFAULT_PARAMETERS.matchToReconstructedCloud);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseAssemblerDfn", parameters.useAssemblerDfn, DEFAULT_PARAMETERS.useAssemblerDfn);
	configurationFilePath = "";
	firstInput = true;

	optionalLeftFilter = NULL;
	optionalRightFilter = NULL;
	reconstructor3d = NULL;
	featuresExtractor3d = NULL;
	registrator3d = NULL;
	cloudAssembler = NULL;
	cloudTransformer = NULL;

	bundleHistory = new BundleHistory(2);
	featureCloud = NewPointCloud();

	#ifdef TESTING
	logFile.open("/InFuse/myLog.txt");
	logFile.close();
	#endif
	}

SparseRegistrationFromStereo::~SparseRegistrationFromStereo()
	{
	DeleteIfNotNull(optionalLeftFilter);
	DeleteIfNotNull(optionalRightFilter);
	DeleteIfNotNull(reconstructor3d);
	DeleteIfNotNull(featuresExtractor3d);
	DeleteIfNotNull(registrator3d);
	DeleteIfNotNull(cloudAssembler);
	DeleteIfNotNull(cloudTransformer);

	DeleteIfNotNull(bundleHistory);
	DeleteIfNotNull(featureCloud);
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
	#ifdef TESTING
	logFile.open("/InFuse/myLog.txt", std::ios::app);
	#endif
	DEBUG_PRINT_TO_LOG("Registration from stereo start", "");

	bundleHistory->AddImages(inLeftImage, inRightImage);

	FrameConstPtr filteredLeftImage = NULL;
	FrameConstPtr filteredRightImage = NULL;
	optionalLeftFilter->Execute(inLeftImage, filteredLeftImage);
	optionalRightFilter->Execute(inRightImage, filteredRightImage);

	PointCloudConstPtr imageCloud = NULL;
	reconstructor3d->Execute(filteredLeftImage, filteredRightImage, imageCloud);

	VisualPointFeatureVector3DConstPtr keypointVector = NULL;
	featuresExtractor3d->Execute(imageCloud, keypointVector);
	DEBUG_SHOW_3D_VISUAL_FEATURES(imageCloud, keypointVector);

	if (!parameters.matchToReconstructedCloud)
		{
		ComputeFeatureCloud(keypointVector); //This will set up featureCloud
		bundleHistory->AddPointCloud(*featureCloud);
		}

	#ifdef TESTING
	logFile << GetNumberOfPoints(*imageCloud) << " ";
	logFile << GetNumberOfPoints(*keypointVector) << " ";
	#endif

	UpdatePose(imageCloud, keypointVector);

	if (!outSuccess)
		{
		bundleHistory->RemoveEntry(0);
		#ifdef TESTING
		logFile << std::endl;
		logFile.close();
		#endif
		return;
		}

	if (outSuccess)
		{
		UpdatePointCloud(imageCloud);
		}

	#ifdef TESTING
	logFile << std::endl;
	logFile.close();
	#endif
	}

void SparseRegistrationFromStereo::setup()
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

const SparseRegistrationFromStereo::RegistrationFromStereoOptionsSet SparseRegistrationFromStereo::DEFAULT_PARAMETERS = 
	{
	/*.searchRadius =*/ 20,
	/*.pointCloudMapResolution =*/ 1e-2,
	/*.matchToReconstructedCloud =*/ false,
	/*.useAssemblerDfn=*/ false
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
	}

void SparseRegistrationFromStereo::InstantiateDFNExecutors()
	{
	optionalLeftFilter = new ImageFilteringExecutor( static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) ) );
	optionalRightFilter = new ImageFilteringExecutor( static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) ) );
	reconstructor3d = new StereoReconstructionExecutor( static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3D") ) );
	featuresExtractor3d = new FeaturesExtraction3DExecutor( static_cast<FeaturesExtraction3DInterface*>( configurator.GetDfn("featuresExtractor3d") ) );
	registrator3d = new Registration3DExecutor( static_cast<Registration3DInterface*>( configurator.GetDfn("registrator3d") ) );
	if (parameters.useAssemblerDfn)
		{
		cloudAssembler = new PointCloudAssemblyExecutor( static_cast<PointCloudAssemblyInterface*>( configurator.GetDfn("cloudAssembler") ) );
		cloudTransformer = new PointCloudTransformExecutor( static_cast<PointCloudTransformInterface*>( configurator.GetDfn("cloudTransformer") ) );
		}
	}

void SparseRegistrationFromStereo::ComputeFeatureCloud(VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr vector)
	{
	ClearPoints(*featureCloud);

	int numberOfPoints = GetNumberOfPoints(*vector);
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		AddPoint(*featureCloud, GetXCoordinate(*vector, pointIndex), GetYCoordinate(*vector, pointIndex), GetZCoordinate(*vector, pointIndex));
		}
	}

#ifdef TESTING
void SparseRegistrationFromStereo::WriteOutputToLogFile()
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

void SparseRegistrationFromStereo::UpdatePose(PointCloudConstPtr imageCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr keypointVector)
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
			pointCloudMap.AddPointCloud( imageCloud, keypointVector, &zeroPose);
			}
		Copy(zeroPose, outPose);
		}
	else
		{
		Pose3DConstPtr poseToPreviousPose = NULL;
		registrator3d->Execute(  featureCloud, bundleHistory->GetPointCloud(1), poseToPreviousPose, outSuccess);
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
				pointCloudMap.AddPointCloud( imageCloud, keypointVector, poseToPreviousPose);
				Copy(*poseToPreviousPose, outPose);
				}
			else
				{
				pointCloudMap.AttachPointCloud( imageCloud, keypointVector, poseToPreviousPose);
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

void SparseRegistrationFromStereo::UpdatePointCloud(PointCloudConstPtr imageCloud)
	{
	PointCloudWrapper::PointCloudConstPtr outputPointCloud = NULL;
	if (parameters.useAssemblerDfn)
		{
		PointCloudConstPtr transformedImageCloud = NULL;
		cloudTransformer->Execute(*imageCloud, outPose, transformedImageCloud);
		cloudAssembler->Execute(*transformedImageCloud, outPose, parameters.searchRadius, outputPointCloud);
		if (parameters.matchToReconstructedCloud)
			{
			VisualPointFeatureVector3DConstPtr keypointVector = NULL;
			featuresExtractor3d->Execute(imageCloud, keypointVector);
			ComputeFeatureCloud(keypointVector); //This will set up featureCloud
			bundleHistory->AddPointCloud(*featureCloud);
			}
		}
	else
		{
		if (parameters.matchToReconstructedCloud)
			{
			VisualPointFeatureVector3DConstPtr fullCloudKeypointVector = pointCloudMap.GetSceneFeaturesVector(&outPose, parameters.searchRadius);
			ComputeFeatureCloud(fullCloudKeypointVector); //This will set up featureCloud
			bundleHistory->AddPointCloud(*featureCloud);
			DeleteIfNotNull(fullCloudKeypointVector);
			}
		outputPointCloud = pointCloudMap.GetScenePointCloudInOrigin(&outPose, parameters.searchRadius);
		}
	Copy(*outputPointCloud, outPointCloud);

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


}
}
}


/** @} */
