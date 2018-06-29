/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CorrectLocalizationTester.cpp
 * @date 03/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the CorrectLocalizationTester class.
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
#include "CorrectLocalizationTester.hpp"
#include<pcl/io/ply_io.h>
#include <ctime>

using namespace dfn_ci;
using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;
using namespace PoseWrapper;
using namespace SupportTypes;

#define DELETE_IF_NOT_NULL(pointer) \
	if (pointer != NULL) \
		{ \
		delete(pointer); \
		}

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
CorrectLocalizationTester::CorrectLocalizationTester() 
	{
	extractorConfigurationFile = "";
	descriptorConfigurationFile = "";
	matcherConfigurationFile = "";
	
	modelCloudFilePath = "";
	sceneCloudFilePath = "";

	extractor = NULL;
	descriptor = NULL;
	matcher = NULL;

	inputSceneCloud = NULL;
	inputModelCloud = NULL;
	inputTruthModelPoseInScene = NULL;
	sceneKeypointsVector = NULL;
	modelKeypointsVector = NULL;
	sceneFeaturesVector = NULL;
	modelFeaturesVector = NULL;
	outputModelPoseInScene = NULL;
	
	outputMatcherSuccess = false;
	dfnsWereConfigured = false;
	inputsWereLoaded = false;
	groundTruthWasLoaded = false;
	}

CorrectLocalizationTester::~CorrectLocalizationTester()
	{
	DELETE_IF_NOT_NULL(inputSceneCloud);
	DELETE_IF_NOT_NULL(inputModelCloud);
	DELETE_IF_NOT_NULL(inputTruthModelPoseInScene);
	DELETE_IF_NOT_NULL(sceneKeypointsVector);
	DELETE_IF_NOT_NULL(modelKeypointsVector);
	DELETE_IF_NOT_NULL(sceneFeaturesVector);
	DELETE_IF_NOT_NULL(modelFeaturesVector);
	DELETE_IF_NOT_NULL(outputModelPoseInScene);
	}

void CorrectLocalizationTester::SetInputClouds(std::string sceneCloudFilePath, std::string modelCloudFilePath, std::string groundTruthPoseFilePath)
	{
	this->sceneCloudFilePath = sceneCloudFilePath;
	this->modelCloudFilePath = modelCloudFilePath;
	this->groundTruthPoseFilePath = groundTruthPoseFilePath;

	LoadPointClouds();
	LoadGroudTruthPose();
	}

void CorrectLocalizationTester::SetConfigurationFiles(std::string extractorConfigurationFile, std::string descriptorConfigurationFile, std::string matcherConfigurationFile)
	{
	this->extractorConfigurationFile = extractorConfigurationFile;
	this->descriptorConfigurationFile = descriptorConfigurationFile;
	this->matcherConfigurationFile = matcherConfigurationFile;

	if (extractor != NULL)
		{
		ConfigureDfns();
		}
	}

void CorrectLocalizationTester::SetDfns(dfn_ci::FeaturesExtraction3DInterface* extractor, dfn_ci::FeaturesDescription3DInterface* descriptor, dfn_ci::FeaturesMatching3DInterface* matcher)
	{
	this->extractor = extractor;
	this->descriptor = descriptor;
	this->matcher = matcher;

	if (extractorConfigurationFile != "")
		{
		ConfigureDfns();
		}
	}

void CorrectLocalizationTester::ExecuteDfns()
	{
	ASSERT(dfnsWereConfigured, "Error: there was a call to ExecuteDfns before actually configuring the DFNs");
	ASSERT(inputsWereLoaded && groundTruthWasLoaded, "Error: there was a call to ExecuteDfns before actually loading inputs");	

	processingTime = 0;
	ExtractFeatures();
	DescribeFeatures();
	MatchFeatures();
	PRINT_TO_LOG("Processing took (seconds): ", processingTime);
	}

bool CorrectLocalizationTester::IsOutputCorrect(float relativeLocationError, float relativeOrientationError, float absoluteLocationError)
	{
	if (!outputMatcherSuccess)
		{
		return false;
		}

	float modelSize = ComputeModelSize();
	float locationError = ComputeLocationError();
	float orientationError = ComputeOrientationError(modelSize);

	PRINT_TO_LOG("The ground truth pose is: ", ToString(*inputTruthModelPoseInScene));
	PRINT_TO_LOG("The model size is: ", modelSize);
	PRINT_TO_LOG("The location error is: ", locationError);
	PRINT_TO_LOG("The orientation error is: ", orientationError);
	PRINT_TO_LOG("Relative location error: ", (locationError / modelSize) );
	PRINT_TO_LOG("Relative orientation error: ", (orientationError / modelSize) );

	bool WithinRelativeLocationError = (locationError <= relativeLocationError * modelSize);
	bool WithinRelatibeOrientationError  = (orientationError <= relativeOrientationError * modelSize);
	bool WithinAbsoluteLocationError = (locationError <= absoluteLocationError);

	if (!WithinRelativeLocationError)
		{
		PRINT_TO_LOG("The location error exceeds the relative Error", relativeLocationError);
		}
	if (!WithinRelatibeOrientationError)
		{
		PRINT_TO_LOG("The orientation error exceeds the relative Error", relativeOrientationError);
		}
	if (!WithinAbsoluteLocationError)
		{
		PRINT_TO_LOG("The location error exceeds the absolute Error", absoluteLocationError);
		}

	return (WithinRelativeLocationError && WithinRelatibeOrientationError && WithinAbsoluteLocationError);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
#define PROCESS_AND_MEASURE_TIME(dfn) \
	{ \
	beginTime = clock(); \
	dfn->process(); \
	endTime = clock(); \
	processingTime += float(endTime - beginTime) / CLOCKS_PER_SEC; \
	}

void CorrectLocalizationTester::ExtractFeatures()
	{
	extractor->pointcloudInput(*inputSceneCloud);
	extractor->process();

	DELETE_IF_NOT_NULL(sceneKeypointsVector);
	VisualPointFeatureVector3DPtr newSceneKeypointsVector = NewVisualPointFeatureVector3D();
	Copy( extractor->featuresOutput(), *newSceneKeypointsVector);
	sceneKeypointsVector = newSceneKeypointsVector;
	PRINT_TO_LOG("Number of scene keypoints extracted is", GetNumberOfPoints(*sceneKeypointsVector));

	extractor->pointcloudInput(*inputModelCloud);
	PROCESS_AND_MEASURE_TIME(extractor);

	DELETE_IF_NOT_NULL(modelKeypointsVector);
	VisualPointFeatureVector3DPtr newModelKeypointsVector = NewVisualPointFeatureVector3D();
	Copy( extractor->featuresOutput(), *newModelKeypointsVector);
	modelKeypointsVector = newModelKeypointsVector;
	PRINT_TO_LOG("Number of model keypoints extracted is", GetNumberOfPoints(*modelKeypointsVector));
	}

void CorrectLocalizationTester::DescribeFeatures()
	{
	descriptor->pointcloudInput(*inputSceneCloud);
	descriptor->featuresInput(*sceneKeypointsVector);
	descriptor->process();

	DELETE_IF_NOT_NULL(sceneFeaturesVector);
	VisualPointFeatureVector3DPtr newSceneFeaturesVector = NewVisualPointFeatureVector3D();	
	Copy( descriptor->featuresOutput(), *newSceneFeaturesVector);
	sceneFeaturesVector = newSceneFeaturesVector;
	PRINT_TO_LOG("Number of scene features described is", GetNumberOfPoints(*sceneFeaturesVector));

	descriptor->pointcloudInput(*inputModelCloud);
	descriptor->featuresInput(*modelKeypointsVector);
	PROCESS_AND_MEASURE_TIME(descriptor);
	DELETE_IF_NOT_NULL(modelFeaturesVector);
	VisualPointFeatureVector3DPtr newModelFeaturesVector = NewVisualPointFeatureVector3D();
	Copy( descriptor->featuresOutput(), *newModelFeaturesVector);
	modelFeaturesVector = newModelFeaturesVector;
	PRINT_TO_LOG("Number of model features described is", GetNumberOfPoints(*modelFeaturesVector));
	}

void CorrectLocalizationTester::MatchFeatures()
	{
	matcher->sourceFeaturesInput(*modelFeaturesVector);
	matcher->sinkFeaturesInput(*sceneFeaturesVector);
	PROCESS_AND_MEASURE_TIME(matcher);

	DELETE_IF_NOT_NULL(outputModelPoseInScene);
	Pose3DPtr newOutputModelPoseInScene = NewPose3D();
	Copy( matcher->transformOutput(), *newOutputModelPoseInScene);
	outputModelPoseInScene = newOutputModelPoseInScene;
	outputMatcherSuccess = matcher->successOutput();

	if (outputMatcherSuccess)
		{
		PRINT_TO_LOG("Matching was successful, the pose found is:", ToString(*outputModelPoseInScene));
		}
	else
		{
		PRINT_TO_LOG("Matching was NOT successful", "");
		}
	}

void CorrectLocalizationTester::LoadPointClouds()
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr baseScenePclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile(sceneCloudFilePath, *baseScenePclCloud);

	DELETE_IF_NOT_NULL(inputSceneCloud);	
	inputSceneCloud = pointCloudConverter.Convert(baseScenePclCloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr baseModelPclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile(modelCloudFilePath, *baseModelPclCloud);

	DELETE_IF_NOT_NULL(inputModelCloud);	
	inputModelCloud = pointCloudConverter.Convert(baseModelPclCloud);

	inputsWereLoaded = true;
	}

void CorrectLocalizationTester::LoadGroudTruthPose()
	{
	std::ifstream file(groundTruthPoseFilePath.c_str());
	ASSERT(file.good(), "Error: groundTruthPoseFilePath is invalid");

	float x, y, z, qx, qy, qz, qw;
	file >> x >> y >> z >> qx >> qy >> qz >> qw;

	Pose3DPtr groundTruthPose = NewPose3D();
	SetPosition(*groundTruthPose, x, y, z);
	SetOrientation(*groundTruthPose, qx, qy, qz, qw);

	file.close();

	DELETE_IF_NOT_NULL(inputTruthModelPoseInScene);
	inputTruthModelPoseInScene = groundTruthPose;
	groundTruthWasLoaded = true;
	}

void CorrectLocalizationTester::ConfigureDfns()
	{
	ASSERT(extractor != NULL && matcher != NULL, "One mandatory DFN was not set up");
	ASSERT(extractorConfigurationFile != "" && matcherConfigurationFile != "", "One mandatory DFN configuration file was not set up");
	ASSERT(descriptor == NULL || descriptorConfigurationFile != "", "Descriptor DFN configuration file was not set up");

	extractor->setConfigurationFile(extractorConfigurationFile);
	extractor->configure();

	if (descriptor != NULL)
		{
		descriptor->setConfigurationFile(descriptorConfigurationFile);
		descriptor->configure();
		}

	matcher->setConfigurationFile(matcherConfigurationFile);
	matcher->configure();

	dfnsWereConfigured = true;
	}

float CorrectLocalizationTester::ComputeLocationError()
	{
	float distanceX = GetXPosition(*outputModelPoseInScene) - GetXPosition(*inputTruthModelPoseInScene);
	float distanceY = GetYPosition(*outputModelPoseInScene) - GetYPosition(*inputTruthModelPoseInScene);
	float distanceZ = GetZPosition(*outputModelPoseInScene) - GetZPosition(*inputTruthModelPoseInScene);
	float squaredDistance = distanceX*distanceX + distanceY*distanceY + distanceZ*distanceZ;

	return std::sqrt(squaredDistance);
	}

float CorrectLocalizationTester::ComputeOrientationError(float modelSize)
	{
	Eigen::Quaternion<float> outputRotation
		(
		GetWOrientation(*outputModelPoseInScene), 
		GetXOrientation(*outputModelPoseInScene), 
		GetYOrientation(*outputModelPoseInScene), 
		GetZOrientation(*outputModelPoseInScene)
		);

	Eigen::Quaternion<float> truthRotation
		(
		GetWOrientation(*inputTruthModelPoseInScene), 
		GetXOrientation(*inputTruthModelPoseInScene), 
		GetYOrientation(*inputTruthModelPoseInScene), 
		GetZOrientation(*inputTruthModelPoseInScene)
		);

	//A point is taken on a sphere of radius modelSize/2; two points are create by rotation this test point by the outputRotation and the truth rotation.
	Eigen::Vector3f testPoint(modelSize/2, 0, 0);
	Eigen::Vector3f outputRotatedPoint = outputRotation * testPoint;
	Eigen::Vector3f truthRotatedPoint = truthRotation * testPoint;

	//The distance between the two points is taken as a measure of the orientation error.
	float differenceX = outputRotatedPoint.x() - truthRotatedPoint.x();
	float differenceY = outputRotatedPoint.y() - truthRotatedPoint.y();
	float differenceZ = outputRotatedPoint.z() - truthRotatedPoint.z();
	float squaredDistance = differenceX*differenceX + differenceY*differenceY + differenceZ*differenceZ;

	return std::sqrt(squaredDistance);
	}

float CorrectLocalizationTester::ComputeModelSize()
	{
	if ( GetNumberOfPoints(*inputModelCloud) == 0)
		{
		return 0;
		}

	//The size of the model is computed by wrapping the model in a tetrahedron and taking the diagonal.
	float minX = GetXCoordinate(*inputModelCloud, 0);
	float minY = GetYCoordinate(*inputModelCloud, 0);
	float minZ = GetZCoordinate(*inputModelCloud, 0);
	float maxX = minX;
	float maxY = minY;
	float maxZ = minZ;

	for(int pointIndex = 1; pointIndex < GetNumberOfPoints(*inputModelCloud); pointIndex++)
		{
		float x = GetXCoordinate(*inputModelCloud, pointIndex);
		float y = GetYCoordinate(*inputModelCloud, pointIndex);
		float z = GetZCoordinate(*inputModelCloud, pointIndex);

		minX = (minX > x) ? x : minX;
		minY = (minY > y) ? y : minY;
		minZ = (minZ > z) ? z : minZ;
		maxX = (maxX < x) ? x : maxX;
		maxY = (maxY < y) ? y : maxY;
		maxZ = (maxZ < z) ? z : maxZ;
		}

	float differenceX = maxX - minX;
	float differenceY = maxY - minY;
	float differenceZ = maxZ - minZ;
	float squaredSize = differenceX*differenceX + differenceY*differenceY + differenceZ*differenceZ;

	return std::sqrt(squaredSize);
	}

/** @} */
