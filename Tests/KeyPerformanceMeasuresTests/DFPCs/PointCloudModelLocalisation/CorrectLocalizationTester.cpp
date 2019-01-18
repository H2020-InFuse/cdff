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

#include <Visualizers/PCLVisualizer.hpp>

using namespace CDFF::DFPC;
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
CorrectLocalizationTester::CorrectLocalizationTester() :
	dfpcConfigurationFilePath(""),
	sceneCloudFilePath(""),
	modelCloudFilePath(""),
	groundTruthPoseFilePath(""),
	beginTime(clock()),
	endTime(beginTime)
	{
	dfpc = NULL;

	inputSceneCloud = NULL;
	inputModelCloud = NULL;
	inputTruthModelPoseInScene = NULL;
	outputModelPoseInScene = NULL;

	outputMatcherSuccess = false;
	dfpcWasConfigured = false;
	inputsWereLoaded = false;
	groundTruthWasLoaded = false;
	
	processingTime = 0;
	}

CorrectLocalizationTester::~CorrectLocalizationTester()
	{
	DELETE_IF_NOT_NULL(inputSceneCloud);
	DELETE_IF_NOT_NULL(inputModelCloud);
	DELETE_IF_NOT_NULL(inputTruthModelPoseInScene);
	DELETE_IF_NOT_NULL(outputModelPoseInScene);
	}

void CorrectLocalizationTester::SetInputClouds(const std::string& sceneCloudFilePath, const std::string& modelCloudFilePath, const std::string& groundTruthPoseFilePath)
	{
	this->sceneCloudFilePath = sceneCloudFilePath;
	this->modelCloudFilePath = modelCloudFilePath;
	this->groundTruthPoseFilePath = groundTruthPoseFilePath;

	LoadPointClouds();
	LoadGroudTruthPose();
	}

void CorrectLocalizationTester::SetDfpc(const std::string& configurationFilePath, CDFF::DFPC::PointCloudModelLocalisationInterface* dfpc)
	{
	this->dfpcConfigurationFilePath = configurationFilePath;
	this->dfpc = dfpc;

	if (dfpcConfigurationFilePath != "")
		{
		ConfigureDfpc();
		}
	}

void CorrectLocalizationTester::ExecuteDfpc(bool showClouds)
	{
	ASSERT(dfpcWasConfigured, "Error: there was a call to ExecuteDfns before actually configuring the DFNs");
	ASSERT(inputsWereLoaded && groundTruthWasLoaded, "Error: there was a call to ExecuteDfns before actually loading inputs");

	processingTime = 0;
	Localize();
	PRINT_TO_LOG("Processing took (seconds): ", processingTime);

	if (showClouds)
		{
		ShowClouds();
		}
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

void CorrectLocalizationTester::Localize()
	{
	dfpc->sceneInput(*inputSceneCloud);
	dfpc->modelInput(*inputModelCloud);
	dfpc->computeModelFeaturesInput(true);
	dfpc->run();

	DELETE_IF_NOT_NULL(outputModelPoseInScene);
	Pose3DPtr newOutputModelPoseInScene = NewPose3D();
	Copy( dfpc->poseOutput(), *newOutputModelPoseInScene );
	outputModelPoseInScene = newOutputModelPoseInScene;
	outputMatcherSuccess = dfpc->successOutput();

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
	baseScenePclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile(sceneCloudFilePath, *baseScenePclCloud);

	DELETE_IF_NOT_NULL(inputSceneCloud);
	inputSceneCloud = pointCloudConverter.Convert(baseScenePclCloud);

	baseModelPclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
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

void CorrectLocalizationTester::ConfigureDfpc()
	{
	ASSERT(dfpc != NULL, "DFPC was not set up");
	ASSERT(dfpcConfigurationFilePath != "", "DFPC configuration file was not set up");

	dfpc->setConfigurationFile(dfpcConfigurationFilePath);
	dfpc->setup();

	dfpcWasConfigured = true;
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

void CorrectLocalizationTester::ShowClouds()
	{
	Eigen::Quaternion<float> rotation(GetWOrientation(*outputModelPoseInScene), -GetXOrientation(*outputModelPoseInScene), 
		-GetYOrientation(*outputModelPoseInScene), -GetZOrientation(*outputModelPoseInScene) );
	Eigen::Translation<float, 3> translation(-GetXPosition(*outputModelPoseInScene), -GetYPosition(*outputModelPoseInScene), -GetZPosition(*outputModelPoseInScene));
	AffineTransform affineTransform = translation * rotation;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedModelCloud(new pcl::PointCloud<pcl::PointXYZ>);
	transformedModelCloud->points.resize( baseModelPclCloud->points.size() );
	for(unsigned pointIndex = 0; pointIndex < baseModelPclCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ transformedPoint = TransformPoint( baseModelPclCloud->points.at(pointIndex), affineTransform );
		transformedModelCloud->points.at(pointIndex) = transformedPoint;
		if (pointIndex == 0)
			{
			std::cout << baseModelPclCloud->points.at(pointIndex).x << " " << baseModelPclCloud->points.at(pointIndex).y << " " << baseModelPclCloud->points.at(pointIndex).z << std::endl;
			std::cout << transformedPoint.x << " " << transformedPoint.y << " " << transformedPoint.z << std::endl;
			}
		}

	std::vector< pcl::PointCloud<pcl::PointXYZ>::ConstPtr > cloudsList = { baseScenePclCloud, transformedModelCloud };
	Visualizers::PclVisualizer::Enable();
	Visualizers::PclVisualizer::ShowPointClouds(cloudsList);
	Visualizers::PclVisualizer::Disable();
	}

pcl::PointXYZ CorrectLocalizationTester::TransformPoint(const pcl::PointXYZ& point, const AffineTransform& affineTransform)
	{
	Eigen::Vector3f eigenPoint(point.x, point.y, point.z);
	Eigen::Vector3f eigenTransformedPoint = affineTransform * eigenPoint;
	pcl::PointXYZ transformedPoint;
	transformedPoint.x = eigenTransformedPoint.x();
	transformedPoint.y = eigenTransformedPoint.y();
	transformedPoint.z = eigenTransformedPoint.z();
	return transformedPoint;
	}

/** @} */
