/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DetectionDescriptionMatching3D.cpp
 * @date 27/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Implementation of the class DetectionDescriptionMatching3DTestInterface
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
#include "DetectionDescriptionMatching3D.hpp"

#include <Executors/FeaturesExtraction3D/FeaturesExtraction3DExecutor.hpp>
#include <Executors/FeaturesDescription3D/FeaturesDescription3DExecutor.hpp>
#include <Executors/FeaturesMatching3D/FeaturesMatching3DExecutor.hpp>

using namespace CDFF::DFN;
using namespace Converters;
using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PoseWrapper;
using namespace SupportTypes;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
DetectionDescriptionMatching3DTestInterface::DetectionDescriptionMatching3DTestInterface(const std::string& folderPath, const std::vector<std::string>& baseConfigurationFileNamesList, 
	const std::string& performanceMeasuresFileName, DFNsSet dfnsSet) : 
	PerformanceTestInterface(folderPath, baseConfigurationFileNamesList, performanceMeasuresFileName),
	groundPositionDistanceAggregator( Aggregator::AVERAGE ),
	groundOrientationDistanceAggregator( Aggregator::AVERAGE )
	{
	extractor = dfnsSet.extractor;
	AddDfn(dfnsSet.extractor);
	descriptor = dfnsSet.descriptor;
	AddDfn(dfnsSet.descriptor);
	matcher = dfnsSet.matcher;
	AddDfn(dfnsSet.matcher);

	AddAggregator("PositionDistance", &groundPositionDistanceAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);
	AddAggregator("AngleDistance", &groundOrientationDistanceAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);

	scenePointCloud = NULL;
	modelPointCloud = NULL;
	sceneFeatureVector = NewVisualPointFeatureVector3D();
	modelPoseInScene = NewPose3D();
	icpSuccess = false;

	inputId = -1;
	inputCloudFile = "../tests/Data/PointClouds/bunny0.ply";
	groundTruthTransformFilePath = "NULL";
	voxelGridFilterSize = 0.001;
	LoadSceneCloud();

	SetPosition(groundTruthPose, 0, 0, 0);
	SetOrientation(groundTruthPose, 0, 0, 0, 1);
	}

DetectionDescriptionMatching3DTestInterface::~DetectionDescriptionMatching3DTestInterface()
	{
	delete(extractor);
	delete(descriptor);
	delete(matcher);
	if (scenePointCloud != NULL)
		{
		delete(scenePointCloud);
		}
	if (modelPointCloud != NULL)
		{
		delete(modelPointCloud);
		}
	delete(sceneFeatureVector);
	delete(modelPoseInScene);
	}

void DetectionDescriptionMatching3DTestInterface::SetInputCloud(const std::string& inputCloudFile, float voxelGridFilterSize)
	{
	this->inputCloudFile = inputCloudFile;
	this->voxelGridFilterSize = voxelGridFilterSize;
	LoadSceneCloud();
	}

void DetectionDescriptionMatching3DTestInterface::SetModelsCloud(std::string groundTruthTransformFilePath, std::vector<std::string> modelsCloudFilesList)
	{
	this->groundTruthTransformFilePath = groundTruthTransformFilePath;
	this->modelsCloudFilesList = modelsCloudFilesList;

	std::ifstream transformsFile( groundTruthTransformFilePath.c_str() );
	ASSERT(transformsFile.good(), "Error transforms file could not be opened");

	while (!transformsFile.eof())
		{
		Pose3D newPose;
		float positionX, positionY, positionZ, rotationX, rotationY, rotationZ, rotationW;
		transformsFile >> positionX;
		transformsFile >> positionY;
		transformsFile >> positionZ;
		transformsFile >> rotationX;
		transformsFile >> rotationY;
		transformsFile >> rotationZ;
		transformsFile >> rotationW;
		if (!transformsFile.eof())
			{
			SetPosition(newPose, positionX, positionY, positionZ);
			SetOrientation(newPose, rotationX, rotationY, rotationZ, rotationW);
			posesList.push_back(newPose);
			}
		}

	transformsFile.close();

	ASSERT(posesList.size() == modelsCloudFilesList.size(), "Poses List size does not match the number of models");
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void DetectionDescriptionMatching3DTestInterface::LoadCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string cloudFile)
	{
	const float EPSILON = 0.00001;
	pcl::PointCloud<pcl::PointXYZ>::Ptr basePclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile(cloudFile, *basePclCloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr finitePointsPclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(unsigned pointIndex = 0; pointIndex < basePclCloud->points.size() && finitePointsPclCloud->points.size() <= MAX_CLOUD_SIZE; pointIndex++)
		{
		pcl::PointXYZ point = basePclCloud->points.at(pointIndex);
		if (point.x == point.x && point.y == point.y && point.z == point.z)
			{
			finitePointsPclCloud->points.push_back(point);
			}
		}

	if (voxelGridFilterSize > EPSILON)
		{
		pcl::VoxelGrid<pcl::PointXYZ> grid;
		grid.setInputCloud(finitePointsPclCloud);
		grid.setLeafSize(voxelGridFilterSize, voxelGridFilterSize, voxelGridFilterSize);
		
		grid.filter(*cloud);
		}
	else
		{
		int cloudSize = finitePointsPclCloud->points.size();
		cloud->points.resize(cloudSize);
		for(int pointIndex = 0; pointIndex < cloudSize; pointIndex++)
			{
			cloud->points.at(pointIndex).x = finitePointsPclCloud->points.at(pointIndex).x;
			cloud->points.at(pointIndex).y = finitePointsPclCloud->points.at(pointIndex).y;
			cloud->points.at(pointIndex).z = finitePointsPclCloud->points.at(pointIndex).z;
			}
		}
	}

void DetectionDescriptionMatching3DTestInterface::LoadSceneCloud()
	{
	sceneCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();	
	LoadCloud(sceneCloud, this->inputCloudFile);
	scenePointCloud = pointCloudConverter.Convert(sceneCloud);
	}

void DetectionDescriptionMatching3DTestInterface::LoadModelCloud(int long inputId)
	{
	if (modelPointCloud != NULL)
		{
		delete(modelPointCloud);
		}

	modelCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	LoadCloud(modelCloud, modelsCloudFilesList.at(inputId));
	modelPointCloud = pointCloudConverter.Convert(modelCloud);

	Copy(posesList.at(inputId), groundTruthPose);
	}

bool DetectionDescriptionMatching3DTestInterface::SetNextInputs()
	{
	inputId++;
	if (inputId >= modelsCloudFilesList.size())
		{
		return false;
		}

	LoadModelCloud(inputId);
	return true;	
	}

void DetectionDescriptionMatching3DTestInterface::ExecuteDfns()
	{
	VisualPointFeatureVector3DConstPtr sceneKeypointVector = NULL;
	Executors::Execute(extractor, scenePointCloud, sceneKeypointVector);
	Executors::Execute(descriptor, scenePointCloud, sceneKeypointVector, sceneFeatureVector);
	numberOfSceneKeypoints = GetNumberOfPoints(*sceneFeatureVector);

	VisualPointFeatureVector3DConstPtr modelKeypointVector = NULL;
	VisualPointFeatureVector3DConstPtr modelFeatureVector = NULL;
	Executors::Execute(extractor, modelPointCloud, modelKeypointVector);
	Executors::Execute(descriptor, modelPointCloud, modelKeypointVector, modelFeatureVector);
	numberOfModelKeypoints = GetNumberOfPoints(*modelFeatureVector);

	Executors::Execute(matcher, modelFeatureVector, sceneFeatureVector, modelPoseInScene, icpSuccess);
	}

DetectionDescriptionMatching3DTestInterface::MeasuresMap DetectionDescriptionMatching3DTestInterface::ExtractMeasures()
	{
	MeasuresMap measuresMap;

	measuresMap["IcpSuccess"] = icpSuccess;
	measuresMap["SceneKeypoints"] = numberOfSceneKeypoints;
	measuresMap["ModelKeypoints"] = numberOfModelKeypoints;
	measuresMap["PositionX"] = GetXPosition(*modelPoseInScene);
	measuresMap["PositionY"] = GetYPosition(*modelPoseInScene);
	measuresMap["PositionZ"] = GetZPosition(*modelPoseInScene);

	if (icpSuccess)
		{
		float positionDistance, angleDistance;
		ComputeDistanceToGroundTruth(positionDistance, angleDistance);

		measuresMap["PositionDistance"] = positionDistance;
		measuresMap["AngleDistance"] = angleDistance;				
		}
	else
		{
		measuresMap["PositionDistance"] = 1;
		measuresMap["AngleDistance"] = 1;
		}

	return measuresMap;
	}

void DetectionDescriptionMatching3DTestInterface::ComputeDistanceToGroundTruth(float& positionDistance, float& angleDistance)
	{
	float differenceX = GetXPosition(groundTruthPose) - GetXPosition(*modelPoseInScene);
	float differenceY = GetYPosition(groundTruthPose) - GetYPosition(*modelPoseInScene);
	float differenceZ = GetZPosition(groundTruthPose) - GetZPosition(*modelPoseInScene);
	float squaredDistance = differenceX*differenceX + differenceY*differenceY + differenceZ*differenceZ;
	positionDistance = std::sqrt(squaredDistance);

	float scalarProduct =
		GetXOrientation(groundTruthPose) * GetXOrientation(*modelPoseInScene) +
		GetYOrientation(groundTruthPose) * GetYOrientation(*modelPoseInScene) +
		GetZOrientation(groundTruthPose) * GetZOrientation(*modelPoseInScene) +
		GetWOrientation(groundTruthPose) * GetWOrientation(*modelPoseInScene);	
	angleDistance = 1 - scalarProduct*scalarProduct;
	}

/** @} */
