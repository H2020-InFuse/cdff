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
#include <pcl/io/ply_io.h>
#include <ctime>

using namespace CDFF::DFN;
using namespace Converters;
using namespace PointCloudWrapper;
using namespace PoseWrapper;

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
CorrectLocalizationTester::CorrectLocalizationTester(const std::string& configurationFile, CDFF::DFN::Registration3DInterface* dfn)
	{
	this->configurationFile = configurationFile;
	this->dfn = dfn;
	ConfigureDfn();

	modelCloudFilePath = "";
	sceneCloudFilePath = "";

	inputSceneCloud = NULL;
	inputModelCloud = NULL;
	inputTruthModelPoseInScene = NULL;
	inputGuessModelPoseInScene = NULL;

	outputMatcherSuccess = false;
	inputsWereLoaded = false;
	groundTruthWasLoaded = false;
	guessPoseWasLoaded = false;
	}

CorrectLocalizationTester::~CorrectLocalizationTester()
	{
	DELETE_IF_NOT_NULL(inputSceneCloud);
	DELETE_IF_NOT_NULL(inputModelCloud);
	DELETE_IF_NOT_NULL(inputTruthModelPoseInScene);
	DELETE_IF_NOT_NULL(inputGuessModelPoseInScene);
	}

void CorrectLocalizationTester::SetInputClouds(const std::string& sceneCloudFilePath, const std::string& modelCloudFilePath, const std::string& groundTruthPoseFilePath)
	{
	this->sceneCloudFilePath = sceneCloudFilePath;
	this->modelCloudFilePath = modelCloudFilePath;
	this->groundTruthPoseFilePath = groundTruthPoseFilePath;

	LoadPointClouds();
	LoadGroudTruthPose();
	}

void CorrectLocalizationTester::SetGuessModelPoseInScene(const std::string& guessPoseFilePath)
	{
	this->guessPoseFilePath = guessPoseFilePath;

	LoadGuessPose();
	}

void CorrectLocalizationTester::ExecuteDfn()
	{
	ASSERT(inputsWereLoaded && groundTruthWasLoaded, "Error: there was a call to ExecuteDfns before actually loading inputs");

	dfn->sourceCloudInput(*inputModelCloud);
	dfn->sinkCloudInput(*inputSceneCloud);
	dfn->useGuessInput(guessPoseWasLoaded);
	if (guessPoseWasLoaded)
		{
		dfn->transformGuessInput(*inputGuessModelPoseInScene);
		}

	clock_t beginTime = clock();
	dfn->process();
	clock_t endTime = clock();
	float processingTime = float(endTime - beginTime) / CLOCKS_PER_SEC;
	PRINT_TO_LOG("Processing took (seconds):", processingTime);

	outputMatcherSuccess = dfn->successOutput();

	if (outputMatcherSuccess)
		{
		const Transform3D& outputModelPoseInScene = dfn->transformOutput();
		PRINT_TO_LOG("Matching was successful, the pose found is:", ToString(outputModelPoseInScene));
		}
	else
		{
		PRINT_TO_LOG("Matching was NOT successful", "");
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
	if (guessPoseWasLoaded)
		{
		PRINT_TO_LOG("The guess pose is: ", ToString(*inputGuessModelPoseInScene));
		}
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

void CorrectLocalizationTester::LoadGuessPose()
	{
	std::ifstream file(guessPoseFilePath.c_str());
	ASSERT(file.good(), "Error: guessPoseFilePath is invalid");

	float x, y, z, qx, qy, qz, qw;
	file >> x >> y >> z >> qx >> qy >> qz >> qw;

	Pose3DPtr guessPose = NewPose3D();
	SetPosition(*guessPose, x, y, z);
	SetOrientation(*guessPose, qx, qy, qz, qw);

	file.close();

	DELETE_IF_NOT_NULL(inputGuessModelPoseInScene);
	inputGuessModelPoseInScene = guessPose;
	guessPoseWasLoaded = true;
	}

void CorrectLocalizationTester::ConfigureDfn()
	{
	dfn->setConfigurationFile(configurationFile);
	dfn->configure();
	}

float CorrectLocalizationTester::ComputeLocationError()
	{
	const Transform3D& outputModelPoseInScene = dfn->transformOutput();
	float distanceX = GetXPosition(outputModelPoseInScene) - GetXPosition(*inputTruthModelPoseInScene);
	float distanceY = GetYPosition(outputModelPoseInScene) - GetYPosition(*inputTruthModelPoseInScene);
	float distanceZ = GetZPosition(outputModelPoseInScene) - GetZPosition(*inputTruthModelPoseInScene);
	float squaredDistance = distanceX*distanceX + distanceY*distanceY + distanceZ*distanceZ;

	return std::sqrt(squaredDistance);
	}

float CorrectLocalizationTester::ComputeOrientationError(float modelSize)
	{
	const Transform3D& outputModelPoseInScene = dfn->transformOutput();
	Eigen::Quaternion<float> outputRotation
		(
		GetWOrientation(outputModelPoseInScene),
		GetXOrientation(outputModelPoseInScene),
		GetYOrientation(outputModelPoseInScene),
		GetZOrientation(outputModelPoseInScene)
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
