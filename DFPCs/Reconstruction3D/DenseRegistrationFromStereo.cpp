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
#include <Visualizers/OpenCVVisualizer.hpp>
#include <Visualizers/PCLVisualizer.hpp>

#include <Executors/ImageFiltering/ImageFilteringExecutor.hpp>
#include <Executors/StereoReconstruction/StereoReconstructionExecutor.hpp>
#include <Executors/Registration3D/Registration3DExecutor.hpp>
#include <Executors/PointCloudAssembly/PointCloudAssemblyExecutor.hpp>
#include <Executors/PointCloudTransform/PointCloudTransformExecutor.hpp>
#include <Executors/PointCloudFiltering/PointCloudFilteringExecutor.hpp>

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
DenseRegistrationFromStereo::DenseRegistrationFromStereo() :
	EMPTY_FEATURE_VECTOR( NewVisualPointFeatureVector3D() )
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
	registrator3d = NULL;
	cloudAssembler = NULL;
	cloudTransformer = NULL;
	cloudFilter = NULL;

	bundleHistory = new BundleHistory(2);

	#ifdef TESTING
	logFile.open("/InFuse/myLog.txt");
	logFile.close();
	#endif
	}

DenseRegistrationFromStereo::~DenseRegistrationFromStereo()
	{
	DeleteIfNotNull(bundleHistory);
	delete( EMPTY_FEATURE_VECTOR );
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
	#ifdef TESTING
	logFile.open("/InFuse/myLog.txt", std::ios::app);
	#endif
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

	if (!parameters.matchToReconstructedCloud)
		{
		bundleHistory->AddPointCloud(*imageCloud);
		}

	#ifdef TESTING
	logFile << GetNumberOfPoints(*imageCloud) << " ";
	#endif

	UpdatePose(imageCloud);

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

void DenseRegistrationFromStereo::setup()
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

const DenseRegistrationFromStereo::RegistrationFromStereoOptionsSet DenseRegistrationFromStereo::DEFAULT_PARAMETERS = 
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
void DenseRegistrationFromStereo::ConfigureExtraParameters()
	{
	parametersHelper.ReadFile( configurator.GetExtraParametersConfigurationFilePath() );

	ASSERT(parameters.pointCloudMapResolution > 0, "DenseRegistrationFromStereo Error, Point Cloud Map resolution is not positive");
	}

void DenseRegistrationFromStereo::InstantiateDFNs()
	{
	optionalLeftFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) );
	optionalRightFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) );
	reconstructor3d = static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3D") );
	registrator3d = static_cast<Registration3DInterface*>( configurator.GetDfn("registrator3d") );
	cloudFilter = static_cast<PointCloudFilteringInterface*>( configurator.GetDfn("cloudFilter", true) );
	if (parameters.useAssemblerDfn)
		{
		cloudAssembler = static_cast<PointCloudAssemblyInterface*>( configurator.GetDfn("cloudAssembler") );
		cloudTransformer = static_cast<PointCloudTransformInterface*>( configurator.GetDfn("cloudTransformer") );
		}
	}

#ifdef TESTING
void DenseRegistrationFromStereo::WriteOutputToLogFile()
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

void DenseRegistrationFromStereo::UpdatePose(PointCloudConstPtr imageCloud)
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
			pointCloudMap.AddPointCloud( imageCloud, EMPTY_FEATURE_VECTOR, &zeroPose);
			}
		Copy(zeroPose, outPose);
		}
	else
		{
		Pose3DConstPtr poseToPreviousPose = NULL;
		Executors::Execute(registrator3d, imageCloud, bundleHistory->GetPointCloud(1), poseToPreviousPose, outSuccess);
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
				pointCloudMap.AddPointCloud( imageCloud, EMPTY_FEATURE_VECTOR, poseToPreviousPose);
				Copy(*poseToPreviousPose, outPose);
				}
			else
				{
				pointCloudMap.AttachPointCloud( imageCloud, EMPTY_FEATURE_VECTOR, poseToPreviousPose);
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

void DenseRegistrationFromStereo::UpdatePointCloud(PointCloudConstPtr imageCloud)
	{
	PointCloudWrapper::PointCloudConstPtr outputPointCloud = NULL;
	if (parameters.useAssemblerDfn)
		{
		PointCloudConstPtr transformedImageCloud = NULL;
		Executors::Execute(cloudTransformer, *imageCloud, outPose, transformedImageCloud);
		Executors::Execute(cloudAssembler, *transformedImageCloud, outPose, parameters.searchRadius, outputPointCloud);
		}
	else
		{
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

}
}
}


/** @} */
