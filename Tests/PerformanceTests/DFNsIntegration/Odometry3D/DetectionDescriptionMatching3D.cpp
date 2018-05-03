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

using namespace dfn_ci;
using namespace Common;
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
DetectionDescriptionMatching3DTestInterface::DetectionDescriptionMatching3DTestInterface(std::string folderPath, std::vector<std::string> baseConfigurationFileNamesList, 
	std::string performanceMeasuresFileName, DFNsSet dfnsSet) : PerformanceTestInterface(folderPath, baseConfigurationFileNamesList, performanceMeasuresFileName)
	{
	extractor = dfnsSet.extractor;
	AddDfn(extractor);
	descriptor = dfnsSet.descriptor;
	AddDfn(descriptor);
	matcher = dfnsSet.matcher;
	AddDfn(matcher);
	SetupMocksAndStubs();

	groundPositionDistanceAggregator = new Aggregator( Aggregator::AVERAGE );
	AddAggregator("PositionDistance", groundPositionDistanceAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);
	groundOrientationDistanceAggregator = new Aggregator( Aggregator::AVERAGE );
	AddAggregator("AngleDistace", groundOrientationDistanceAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);

	scenePointCloud = NULL;
	modelPointCloud = NULL;
	sceneKeypointsVector = NULL;
	modelKeypointsVector = NULL;
	sceneFeaturesVector = NULL;
	modelFeaturesVector = NULL;
	modelPoseInScene = NULL;
	icpSuccess = false;

	inputId = -1;
	inputCloudFile = "../tests/Data/PointClouds/bunny0.ply";
	groundTruthTransformFilePath = "NULL";
	voxelGridFilterSize = 0.001;
	LoadSceneCloud();
	}

DetectionDescriptionMatching3DTestInterface::~DetectionDescriptionMatching3DTestInterface()
	{
	delete(stubNormalsCache);
	delete(mockNormalsConverter);
	delete(stubPointCloudCache);
	delete(mockPointCloudConverter);
	delete(stubFeatures3dCache);
	delete(mockFeatures3dConverter);
	delete(stubFeaturesCloudCache);
	delete(mockFeaturesCloudConverter);
	delete(stubTransformCache);
	delete(mockTransformConverter);

	if (scenePointCloud != NULL)
		{
		delete(scenePointCloud);
		}
	if (modelPointCloud != NULL)
		{
		delete(modelPointCloud);
		}
	if (sceneKeypointsVector != NULL)
		{
		delete(sceneKeypointsVector);
		}
	if (modelKeypointsVector != NULL)
		{
		delete(modelKeypointsVector);
		}
	if (sceneFeaturesVector != NULL)
		{
		delete(sceneFeaturesVector);
		}
	if (modelFeaturesVector != NULL)
		{
		delete(modelFeaturesVector);
		}
	if (modelPoseInScene != NULL)
		{
		delete(modelPoseInScene);
		}
	}

void DetectionDescriptionMatching3DTestInterface::SetInputCloud(std::string inputCloudFile, float voxelGridFilterSize)
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
		SetPosition(newPose, positionX, positionY, positionZ);
		SetOrientation(newPose, rotationX, rotationY, rotationZ, rotationW);
		posesList.push_back(newPose);
		}

	transformsFile.close();
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
		cloud = finitePointsPclCloud;
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

	SetPosition(groundTruthPose, 0, 0, 0);
	SetOrientation(groundTruthPose, 0, 0, 0, 1);
	}

void DetectionDescriptionMatching3DTestInterface::SetupMocksAndStubs()
	{
	stubNormalsCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr>();
	mockNormalsConverter = new Mocks::PointCloudToPclNormalsCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr, PointCloudToPclNormalsCloudConverter>::Instance(stubNormalsCache, mockNormalsConverter);

	stubPointCloudCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>();
	mockPointCloudConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubPointCloudCache, mockPointCloudConverter);

	stubFeatures3dCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>();
	mockFeatures3dConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Instance(stubFeatures3dCache, mockFeatures3dConverter);

	stubFeaturesCloudCache = new Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures>();
	mockFeaturesCloudConverter = new Mocks::VisualPointFeatureVector3DToPclPointCloudConverter();
	ConversionCache<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures, VisualPointFeatureVector3DToPclPointCloudConverter>::Instance(stubFeaturesCloudCache, mockFeaturesCloudConverter);

	stubTransformCache = new Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>();
	mockTransformConverter = new Mocks::EigenTransformToTransform3DConverter();
	ConversionCache<Eigen::Matrix4f, Transform3DConstPtr, EigenTransformToTransform3DConverter>::Instance(stubTransformCache, mockTransformConverter);
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
	if (sceneKeypointsVector != NULL)
		{
		delete(sceneKeypointsVector);
		}
	extractor->pointCloudInput(scenePointCloud);
	extractor->process();
	sceneKeypointsVector = extractor->featuresSetOutput();

	if (modelKeypointsVector != NULL)
		{
		delete(modelKeypointsVector);
		}
	extractor->pointCloudInput(modelPointCloud);
	extractor->process();
	modelKeypointsVector = extractor->featuresSetOutput();

	if (sceneFeaturesVector != NULL)
		{
		delete(sceneFeaturesVector);
		}
	descriptor->pointCloudInput(scenePointCloud);
	descriptor->featuresSetInput(sceneKeypointsVector);
	descriptor->process();
	sceneFeaturesVector = descriptor->featuresSetWithDescriptorsOutput();

	if (modelFeaturesVector != NULL)
		{
		delete(modelFeaturesVector);
		}
	descriptor->pointCloudInput(modelPointCloud);
	descriptor->featuresSetInput(modelKeypointsVector);
	descriptor->process();
	modelFeaturesVector = descriptor->featuresSetWithDescriptorsOutput();

	if (modelPoseInScene != NULL)
		{
		delete(modelPoseInScene);
		}
	matcher->sourceFeaturesVectorInput(modelFeaturesVector);
	matcher->sinkFeaturesVectorInput(sceneFeaturesVector);
	matcher->process();
	icpSuccess = matcher->successOutput();
	modelPoseInScene = matcher->transformOutput();
	}

DetectionDescriptionMatching3DTestInterface::MeasuresMap DetectionDescriptionMatching3DTestInterface::ExtractMeasures()
	{
	MeasuresMap measuresMap;

	measuresMap["IcpSuccess"] = icpSuccess;
	measuresMap["SceneKeypoints"] = GetNumberOfPoints(*sceneKeypointsVector);
	measuresMap["ModelKeypoints"] = GetNumberOfPoints(*modelKeypointsVector);

	if (icpSuccess)
		{
		float positionDistance, angleDistance;
		ComputeDistanceToGroundTruth(positionDistance, angleDistance);

		measuresMap["PositionDistance"] = positionDistance;
		measuresMap["AngleDistace"] = angleDistance;				
		}
	else
		{
		measuresMap["PositionDistance"] = 1;
		measuresMap["AngleDistace"] = 1;
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
