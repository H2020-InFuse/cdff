/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PositionTester.cpp
 * @date 19/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the PositionTester class.
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
#include "PositionTester.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

using namespace dfn_ci;
using namespace Converters;
using namespace CorrespondenceMap2DWrapper;
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
PositionTester::PositionTester(std::string configurationFilePath, dfn_ci::BundleAdjustmentInterface* dfn) 
	{
	this->configurationFilePath = configurationFilePath;
	this->dfn = dfn;

	inputCorrespondenceMapsSequence = NULL;
	inputPoseReferenceSequence = NULL;
	outputPosesSequence = NewPoses3DSequence();

	bundleAdjustmentSuccess = false;
	correspondencesWereLoaded = false;
	positionReferencesWereLoaded = false;

	ConfigureDfn();
	}

PositionTester::~PositionTester()
	{
	DELETE_IF_NOT_NULL(inputCorrespondenceMapsSequence);
	DELETE_IF_NOT_NULL(inputPoseReferenceSequence);
	delete(outputPosesSequence);
	}

void PositionTester::SetFilesPaths(std::string inputCorrespondenceFilePath, std::string positionReferenceFilePath)
	{
	this->inputCorrespondenceFilePath = inputCorrespondenceFilePath;
	this->positionReferenceFilePath = positionReferenceFilePath;

	LoadCorrespondences();
	correspondencesWereLoaded = true;
	LoadPositionReferences();
	positionReferencesWereLoaded = true;
	}

void PositionTester::ExecuteDfn()
	{
	dfn->correspondenceMapsSequenceInput(*inputCorrespondenceMapsSequence);
	dfn->process();
	Copy( dfn->posesSequenceOutput(), *outputPosesSequence);
	}

bool PositionTester::ArePositionsCloseToReference(float relativeLocationError, float relativeOrientationError, float modelSize)
	{
	ASSERT(correspondencesWereLoaded && positionReferencesWereLoaded, "Error: some inputs were not correctly loaded");
	if (!bundleAdjustmentSuccess)
		{
		PRINT_TO_LOG("Bundle Adjustment did not succeed", "");
		return false;
		}

	float locationError = ComputeLocationError();
	float orientationError = ComputeOrientationError(modelSize);

	PRINT_TO_LOG("The model size is: ", modelSize);
	PRINT_TO_LOG("The location error is: ", locationError);
	PRINT_TO_LOG("The orientation error is: ", orientationError);
	PRINT_TO_LOG("Relative location error: ", (locationError / modelSize) );
	PRINT_TO_LOG("Relative orientation error: ", (orientationError / modelSize) );

	bool WithinRelativeLocationError = (locationError <= relativeLocationError * modelSize);
	bool WithinRelatibeOrientationError  = (orientationError <= relativeOrientationError * modelSize);

	if (!WithinRelativeLocationError)
		{
		PRINT_TO_LOG("The location error exceeds the relative Error", relativeLocationError);
		}
	if (!WithinRelatibeOrientationError)
		{
		PRINT_TO_LOG("The orientation error exceeds the relative Error", relativeOrientationError);
		}

	return (WithinRelativeLocationError && WithinRelatibeOrientationError);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void PositionTester::LoadCorrespondences()
	{
	cv::FileStorage opencvFile(inputCorrespondenceFilePath, cv::FileStorage::READ);

	cv::Mat cvMeasurementMap;
	opencvFile["MeasurementMap"] >> cvMeasurementMap;
	opencvFile.release();

	DELETE_IF_NOT_NULL(inputCorrespondenceMapsSequence);
	inputCorrespondenceMapsSequence = correspondenceConverter.Convert(cvMeasurementMap);
	}

void PositionTester::LoadPositionReferences()
	{
	std::ifstream file(positionReferenceFilePath.c_str());
	ASSERT(file.good(), "Error: positionReferenceFilePath is invalid");

	DELETE_IF_NOT_NULL(inputPoseReferenceSequence);
	inputPoseReferenceSequence = NewPoses3DSequence();

	while(file.good())
		{
		float x, y, z, qx, qy, qz, qw;
		file >> x >> y >> z >> qx >> qy >> qz >> qw;

		if (file.good())
			{
			Pose3DPtr groundTruthPose = NewPose3D();
			SetPosition(*groundTruthPose, x, y, z);
			SetOrientation(*groundTruthPose, qx, qy, qz, qw);
			AddPose(*inputPoseReferenceSequence, *groundTruthPose);
			delete(groundTruthPose);
			}
		}
	file.close();
	}

void PositionTester::ConfigureDfn()
	{
	dfn->setConfigurationFile(configurationFilePath);
	dfn->configure();
	}

float PositionTester::ComputeLocationError()
	{
	ASSERT( GetNumberOfPoses(*outputPosesSequence) == GetNumberOfPoses(*inputPoseReferenceSequence), "Error: reference pose number is not the same as output pose number");

	int numberOfPoses = GetNumberOfPoses(*outputPosesSequence);
	float totalError = 0;
	for(int poseIndex = 0; poseIndex < numberOfPoses; poseIndex++)
		{
		totalError += ComputeLocationError(poseIndex);
		}
	float averageError = totalError / static_cast<float>(numberOfPoses);
	return averageError;
	}

float PositionTester::ComputeLocationError(float poseIndex)
	{
	const Pose3D& outputPose = GetPose(*outputPosesSequence, poseIndex);
	const Pose3D& referencePose = GetPose(*inputPoseReferenceSequence, poseIndex);

	float distanceX = GetXPosition(outputPose) - GetXPosition(referencePose);
	float distanceY = GetYPosition(outputPose) - GetYPosition(referencePose);
	float distanceZ = GetZPosition(outputPose) - GetZPosition(referencePose);
	float squaredDistance = distanceX*distanceX + distanceY*distanceY + distanceZ*distanceZ;

	return std::sqrt(squaredDistance);
	}

float PositionTester::ComputeOrientationError(float modelSize)
	{
	ASSERT( GetNumberOfPoses(*outputPosesSequence) == GetNumberOfPoses(*inputPoseReferenceSequence), "Error: reference pose number is not the same as output pose number");

	int numberOfPoses = GetNumberOfPoses(*outputPosesSequence);
	float totalError = 0;
	for(int poseIndex = 0; poseIndex < numberOfPoses; poseIndex++)
		{
		totalError += ComputeOrientationError(poseIndex, modelSize);
		}
	float averageError = totalError / static_cast<float>(numberOfPoses);
	return averageError;
	}

float PositionTester::ComputeOrientationError(float poseIndex, float modelSize)
	{
	const Pose3D& outputPose = GetPose(*outputPosesSequence, poseIndex);
	const Pose3D& referencePose = GetPose(*inputPoseReferenceSequence, poseIndex);

	Eigen::Quaternion<float> outputRotation
		(
		GetWOrientation(outputPose), 
		GetXOrientation(outputPose), 
		GetYOrientation(outputPose), 
		GetZOrientation(outputPose)
		);

	Eigen::Quaternion<float> referenceRotation
		(
		GetWOrientation(referencePose), 
		GetXOrientation(referencePose), 
		GetYOrientation(referencePose), 
		GetZOrientation(referencePose)
		);

	//A point is taken on a sphere of radius modelSize/2; two points are create by rotation this test point by the outputRotation and the truth rotation.
	Eigen::Vector3f testPoint(modelSize/2, 0, 0);
	Eigen::Vector3f outputRotatedPoint = outputRotation * testPoint;
	Eigen::Vector3f referenceRotatedPoint = referenceRotation * testPoint;

	//The distance between the two points is taken as a measure of the orientation error.
	float differenceX = outputRotatedPoint.x() - referenceRotatedPoint.x();
	float differenceY = outputRotatedPoint.y() - referenceRotatedPoint.y();
	float differenceZ = outputRotatedPoint.z() - referenceRotatedPoint.z();
	float squaredDistance = differenceX*differenceX + differenceY*differenceY + differenceZ*differenceZ;

	return std::sqrt(squaredDistance);
	}

/** @} */
