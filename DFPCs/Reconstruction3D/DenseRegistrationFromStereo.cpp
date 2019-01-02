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

#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>
#include <pcl/io/ply_io.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <pcl/kdtree/kdtree_flann.h>

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
	#define ADD_PARAMETER_WITH_HELPER(type, helperType, groupName, parameterName, parameterVariable) \
		parametersHelper.AddParameter<type, helperType>(groupName, parameterName, parameters.parameterVariable, DEFAULT_PARAMETERS.parameterVariable);

	parameters = DEFAULT_PARAMETERS;

	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudMapResolution", parameters.pointCloudMapResolution, DEFAULT_PARAMETERS.pointCloudMapResolution);
	parametersHelper.AddParameter<float>("GeneralParameters", "SearchRadius", parameters.searchRadius, DEFAULT_PARAMETERS.searchRadius);
	parametersHelper.AddParameter<bool>("GeneralParameters", "MatchToReconstructedCloud", parameters.matchToReconstructedCloud, DEFAULT_PARAMETERS.matchToReconstructedCloud);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseAssemblerDfn", parameters.useAssemblerDfn, DEFAULT_PARAMETERS.useAssemblerDfn);

	ADD_PARAMETER_WITH_HELPER(CloudUpdateType, CloudUpdateTypeHelper, "GeneralParameters", "CloudUpdateType", cloudUpdateType);
	parametersHelper.AddParameter<int>("GeneralParameters", "CloudUpdateTime", parameters.cloudUpdateTime, DEFAULT_PARAMETERS.cloudUpdateTime);
	parametersHelper.AddParameter<double>("GeneralParameters", "CloudUpdateTranslationDistance", parameters.cloudUpdateTranslationDistance, DEFAULT_PARAMETERS.cloudUpdateTranslationDistance);
	parametersHelper.AddParameter<double>("GeneralParameters", "CloudUpdateOrientationDistance", parameters.cloudUpdateOrientationDistance, DEFAULT_PARAMETERS.cloudUpdateOrientationDistance);
	parametersHelper.AddParameter<float>("GeneralParameters", "OverlapThreshold", parameters.overlapThreshold, DEFAULT_PARAMETERS.overlapThreshold);
	parametersHelper.AddParameter<float>("GeneralParameters", "OverlapInlierDistance", parameters.overlapInlierDistance, DEFAULT_PARAMETERS.overlapInlierDistance);

	parametersHelper.AddParameter<bool>("GeneralParameters", "SaveCloudsToFile", parameters.saveCloudsToFile, DEFAULT_PARAMETERS.saveCloudsToFile);
	parametersHelper.AddParameter<int>("GeneralParameters", "CloudSaveTime", parameters.cloudSaveTime, DEFAULT_PARAMETERS.cloudSaveTime);
	parametersHelper.AddParameter<std::string>("GeneralParameters", "CloudSavePath", parameters.cloudSavePath, DEFAULT_PARAMETERS.cloudSavePath);

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
	outputPoseAtLastMergeSet = false;
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

	UpdatePose(imageCloud);
	if (parameters.cloudUpdateType == CloudUpdateType::TimePassed)
		{
		UpdatePointCloudOnTimePassed(imageCloud);
		}
	else if (parameters.cloudUpdateType == CloudUpdateType::DistanceCovered)
		{
		UpdatePointCloudOnDistanceCovered(imageCloud);
		}
	else
		{
		UpdatePointCloudOnMaximumOverlapping(imageCloud);
		}

	SaveOutputCloud();
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
	/*.useAssemblerDfn=*/ false,
	/*.cloudUpdateType=*/ CloudUpdateType::TimePassed,
	/*.cloudUpdateTime=*/ 50,
	/*.cloudUpdateTranslationDistance=*/ 0.5,
	/*.cloudUpdateOrientationDistance=*/ 0.5,
	/*.overlapThreshold=*/ 0.80,
	/*.overlapInlierDistance=*/ 0.01,
	/*.saveCloudsToFile=*/ false,
	/*.cloudSaveTime=*/ 100,
	/*.cloudSavePath=*/ ""
	};

DenseRegistrationFromStereo::CloudUpdateTypeHelper::CloudUpdateTypeHelper
	(const std::string& parameterName, CloudUpdateType& boundVariable, const CloudUpdateType& defaultValue) :
	ParameterHelper(parameterName, boundVariable, defaultValue)
{
}

DenseRegistrationFromStereo::CloudUpdateType DenseRegistrationFromStereo::CloudUpdateTypeHelper::Convert(const std::string& cloudUpdateType)
{
	if (cloudUpdateType == "Time" || cloudUpdateType == "0")
	{
		return CloudUpdateType::TimePassed;
	}
	if (cloudUpdateType == "Distance" || cloudUpdateType == "1")
	{
		return CloudUpdateType::DistanceCovered;
	}
	else if (cloudUpdateType == "Overlapping" || cloudUpdateType == "2")
	{
		return CloudUpdateType::MaximumOverlapping;
	}
	else
	{
		std::string errorString = "DenseRegistrationFromStereo ConfigurationError: cloudUpdateType has to be one of ";
		errorString += "{Time, Distance, Overlapping}";
		ASSERT(false, errorString);
	}
}

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
	ASSERT(parameters.cloudUpdateTime > 0, "DenseRegistrationFromStereo Error, cloudUpdateTime is not positive");
	ASSERT(parameters.cloudSaveTime > 0, "DenseRegistrationFromStereo Error, cloudUpdateTime is not positive");
	ASSERT(parameters.cloudUpdateTranslationDistance > 0, "DenseRegistrationFromStereo Error, cloudUpdateTranslationDistance is not positive");
	ASSERT(parameters.cloudUpdateOrientationDistance > 0, "DenseRegistrationFromStereo Error, cloudUpdateOrientationDistance is not positive");
	ASSERT(parameters.overlapThreshold > 0, "DenseRegistrationFromStereo Error, overlapThreshold is not positive");
	ASSERT(parameters.overlapInlierDistance > 0, "DenseRegistrationFromStereo Error, overlapInlierDistance is not positive");
	if (parameters.saveCloudsToFile)
		{
		ASSERT( access(parameters.cloudSavePath.c_str(), 0) == 0, "DenseRegistrationFromStereo Error, save folder does not exists");
		struct stat status;
		stat(parameters.cloudSavePath.c_str(), &status);
		ASSERT(status.st_mode & S_IFDIR, "DenseRegistrationFromStereo Error, save folder is not a valid directory");
		}
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
		Executors::Execute(registrator3d, imageCloud, bundleHistory->GetPointCloud(1), &outPose, poseToPreviousPose, outSuccess);
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
		}
	}

void DenseRegistrationFromStereo::UpdatePointCloudOnTimePassed(PointCloudWrapper::PointCloudConstPtr imageCloud)
	{
	static int mergeCounter = 0;
	if (!outSuccess)
		{
		bundleHistory->RemoveEntry(0);
		}
	else
		{
		if (mergeCounter == 0)
			{
			float translationDistance = ComputeTranslationDistance(outPose, outputPoseAtLastMerge);
			float orientationDistance = ComputeOrientationDistance(outPose, outputPoseAtLastMerge);
			PRINT_TO_LOG("translationDistance", translationDistance);
			PRINT_TO_LOG("orientationDistance", orientationDistance);

			static bool first = true;
			if (!first)
				{
				float overlappingRatio = ComputeOverlappingRatio(imageCloud, outPose, bundleHistory->GetPointCloud(1));
				PRINT_TO_LOG("overlappingRation", overlappingRatio);
				}
			first = false;
			MergePointCloud(imageCloud);
			Copy(outPose, outputPoseAtLastMerge);
			}
		else
			{
			bundleHistory->AddPointCloud(outPointCloud);
			}
		}
	mergeCounter = (mergeCounter + 1) % parameters.cloudUpdateTime;
	}

void DenseRegistrationFromStereo::UpdatePointCloudOnDistanceCovered(PointCloudWrapper::PointCloudConstPtr imageCloud)
	{
	if (!outSuccess)
		{
		bundleHistory->RemoveEntry(0);
		}
	else
		{
		float translationDistance = ComputeTranslationDistance(outPose, outputPoseAtLastMerge);
		float orientationDistance = ComputeOrientationDistance(outPose, outputPoseAtLastMerge);
		if (!outputPoseAtLastMergeSet || translationDistance > parameters.cloudUpdateTranslationDistance || orientationDistance > parameters.cloudUpdateOrientationDistance)
			{
			MergePointCloud(imageCloud);
			Copy(outPose, outputPoseAtLastMerge);
			outputPoseAtLastMergeSet = true;
			}
		else
			{
			bundleHistory->AddPointCloud(outPointCloud);
			}
		}
	}

void DenseRegistrationFromStereo::UpdatePointCloudOnMaximumOverlapping(PointCloudWrapper::PointCloudConstPtr inputCloud)
	{
	if (!outSuccess)
		{
		bundleHistory->RemoveEntry(0);
		}
	else
		{
		static bool firstCloud = true;
		float overlappingRatio = 0;
		if (!firstCloud)
			{
			overlappingRatio = ComputeOverlappingRatio(inputCloud, outPose, bundleHistory->GetPointCloud(1));
			}
		if (firstCloud || overlappingRatio < parameters.overlapThreshold)
			{
			MergePointCloud(inputCloud);
			firstCloud = false;
			}
		else
			{
			bundleHistory->AddPointCloud(outPointCloud);
			}
		}
	}

void DenseRegistrationFromStereo::MergePointCloud(PointCloudConstPtr imageCloud)
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

	DEBUG_SHOW_POINT_CLOUD(outputPointCloud);
	if (!parameters.useAssemblerDfn)
		{
		DeleteIfNotNull(outputPointCloud);
		}
	}

void DenseRegistrationFromStereo::SaveOutputCloud()
	{
	if (!parameters.saveCloudsToFile)
		{
		return;
		}
	
	static int saveCounter = 0;
	if (saveCounter == 0)
		{
		static int fileNumber = 0;
		std::string outputFilePath = parameters.cloudSavePath + "/intermediate_output_" + std::to_string(fileNumber) + ".ply";
		Converters::PointCloudToPclPointCloudConverter inConverter;		
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr transformedCloud = inConverter.Convert( bundleHistory->GetPointCloud(0) );
		pcl::PLYWriter writer;
		writer.write(outputFilePath, *transformedCloud, true);
		fileNumber += parameters.cloudSaveTime;
		}
	saveCounter = (saveCounter + 1) % parameters.cloudSaveTime;
	}

float DenseRegistrationFromStereo::ComputeOverlappingRatio(PointCloudConstPtr cloud, const Pose3D& pose, PointCloudConstPtr sceneCloud)
	{
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclSceneCloud = pointCloudToPclPointCloudConverter.Convert(sceneCloud);

	Eigen::Quaternion<float> rotation(GetWOrientation(pose), GetXOrientation(pose),	GetYOrientation(pose), GetZOrientation(pose) );
	Eigen::Translation<float, 3> translation(GetXPosition(pose), GetYPosition(pose), GetZPosition(pose));
	AffineTransform affineTransform = translation * rotation;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(pclSceneCloud);

	std::vector<int> indexList;
	std::vector<float> squaredDistanceList;

	int numberOfPoints = GetNumberOfPoints(*cloud);
	int overlappingCounter = 0;
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		pcl::PointXYZ searchPoint( GetXCoordinate(*cloud, pointIndex), GetYCoordinate(*cloud, pointIndex), GetZCoordinate(*cloud, pointIndex) );
		pcl::PointXYZ transformedSearchPoint = TransformPoint( searchPoint, affineTransform );
		kdtree.radiusSearch( transformedSearchPoint, parameters.overlapInlierDistance, indexList, squaredDistanceList );
		if (indexList.size() > 0)
			{
			overlappingCounter++;
			}
		}

	return ( static_cast<float>(overlappingCounter) / static_cast<float>(numberOfPoints) );
	}

pcl::PointXYZ DenseRegistrationFromStereo::TransformPoint(const pcl::PointXYZ& point, const AffineTransform& affineTransform)
	{
	Eigen::Vector3f eigenPoint(point.x, point.y, point.z);
	Eigen::Vector3f eigenTransformedPoint = affineTransform * eigenPoint;
	pcl::PointXYZ transformedPoint;
	transformedPoint.x = eigenTransformedPoint.x();
	transformedPoint.y = eigenTransformedPoint.y();
	transformedPoint.z = eigenTransformedPoint.z();
	return transformedPoint;
	}

}
}
}


/** @} */
