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

	configurationFilePath = "";
	firstInput = true;

	optionalLeftFilter = NULL;
	optionalRightFilter = NULL;
	reconstructor3d = NULL;
	featuresExtractor3d = NULL;
	optionalFeaturesDescriptor3d = NULL;
	featuresMatcher3d = NULL;

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

	PointCloudConstPtr imageCloud = NULL;
	reconstructor3d->Execute(filteredLeftImage, filteredRightImage, imageCloud);

	VisualPointFeatureVector3DConstPtr keypointVector = NULL;
	featuresExtractor3d->Execute(imageCloud, keypointVector);
	DEBUG_SHOW_3D_VISUAL_FEATURES(imageCloud, keypointVector);

	VisualPointFeatureVector3DConstPtr featureVector = NULL;
	optionalFeaturesDescriptor3d->Execute(imageCloud, keypointVector, featureVector);
	DEBUG_PRINT_TO_LOG("Described Features:", GetNumberOfPoints(*featureVector));

	if (!parameters.matchToReconstructedCloud)
		{
		bundleHistory->AddFeatures3d(*featureVector);
		}

	#ifdef TESTING
	logFile << GetNumberOfPoints(*imageCloud) << " ";
	logFile << GetNumberOfPoints(*keypointVector) << " ";
	logFile << GetNumberOfPoints(*featureVector) << " ";
	#endif

	if (firstInput)
		{
		firstInput = false;
		outSuccess = true;

		Pose3D zeroPose;
		SetPosition(zeroPose, 0, 0, 0);
		SetOrientation(zeroPose, 0, 0, 0, 1);
		pointCloudMap.AddPointCloud( imageCloud, featureVector, &zeroPose);
		}
	else
		{
		Pose3DConstPtr poseToPreviousPose = NULL;
		featuresMatcher3d->Execute( featureVector, bundleHistory->GetFeatures3d(1), poseToPreviousPose, outSuccess);
		if (outSuccess)
			{
			pointCloudMap.AttachPointCloud( imageCloud, featureVector, poseToPreviousPose);
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
		Copy( pointCloudMap.GetLatestPose(), outPose);
		if (parameters.matchToReconstructedCloud)
			{
			VisualPointFeatureVector3DConstPtr fullCloudFeatureVector = pointCloudMap.GetSceneFeaturesVector(&outPose, parameters.searchRadius);
			bundleHistory->AddFeatures3d(*fullCloudFeatureVector);
			DeleteIfNotNull(fullCloudFeatureVector);
			}
		PointCloudWrapper::PointCloudConstPtr outputPointCloud = pointCloudMap.GetScenePointCloudInOrigin(&outPose, parameters.searchRadius);
		Copy(*outputPointCloud, outPointCloud); 

		DEBUG_PRINT_TO_LOG("pose", ToString(outPose));
		DEBUG_PRINT_TO_LOG("points", GetNumberOfPoints(*outputPointCloud));

		#ifdef TESTING
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
		#endif

		DEBUG_SHOW_POINT_CLOUD(outputPointCloud);
		DeleteIfNotNull(outputPointCloud);
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
	.searchRadius = 20,
	.pointCloudMapResolution = 1e-2,
	.matchToReconstructedCloud = false
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
	}

}
}
}


/** @} */
