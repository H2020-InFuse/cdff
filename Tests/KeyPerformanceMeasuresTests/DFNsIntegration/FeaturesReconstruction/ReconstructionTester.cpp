/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ReconstructionTester.cpp
 * @date 15/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the ReconstructionTester class.
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
#include "ReconstructionTester.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <ctime>

using namespace CDFF::DFN::FundamentalMatrixComputation;
using namespace CDFF::DFN::CamerasTransformEstimation;
using namespace CDFF::DFN::PointCloudReconstruction2DTo3D;
using namespace PointCloudWrapper;
using namespace MatrixWrapper;
using namespace PoseWrapper;
using namespace CorrespondenceMap2DWrapper;

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
ReconstructionTester::ReconstructionTester() 
	{
	fundamentalMatrixEstimatorFilePath = "";
	poseEstimatorConfigurationFilePath = "";
	reconstructorConfigurationFilePath = "";

	inputCorrespondenceMap = NULL;
	fundamentalMatrix = NULL;
	cameraTransform = NULL;
	outputPointCloud = NULL;

	poseEstimator = NULL;
	reconstructor = NULL;

	dfnsWereLoaded = false;
	inputCorrespondencesWereLoaded = false;
	dfnsWereExecuted = false;
	}

ReconstructionTester::~ReconstructionTester()
	{
	DELETE_IF_NOT_NULL(inputCorrespondenceMap);
	DELETE_IF_NOT_NULL(fundamentalMatrix);
	DELETE_IF_NOT_NULL(cameraTransform);
	DELETE_IF_NOT_NULL(outputPointCloud);
	}

void ReconstructionTester::SetDfns(FundamentalMatrixComputationInterface* fundamentalMatrixEstimator, 
				CamerasTransformEstimationInterface* poseEstimator, PointCloudReconstruction2DTo3DInterface* reconstructor)
	{
	this->fundamentalMatrixEstimator = fundamentalMatrixEstimator;
	this->poseEstimator = poseEstimator;
	this->reconstructor = reconstructor;

	if (poseEstimatorConfigurationFilePath != "")
		{
		ConfigureDfns();
		}
	}

void ReconstructionTester::SetConfigurationFilePaths(std::string fundamentalMatrixEstimatorFilePath, std::string poseEstimatorConfigurationFilePath, std::string reconstructorConfigurationFilePath)
	{
	this->fundamentalMatrixEstimatorFilePath = fundamentalMatrixEstimatorFilePath;
	this->poseEstimatorConfigurationFilePath = poseEstimatorConfigurationFilePath;
	this->reconstructorConfigurationFilePath = reconstructorConfigurationFilePath;

	if (poseEstimator != NULL && reconstructor != NULL)
		{
		ConfigureDfns();
		}
	}

void ReconstructionTester::SetInputFilePath(std::string inputCorrespodencesFilePath)
	{
	this->inputCorrespodencesFilePath = inputCorrespodencesFilePath;

	LoadInputCorrespondences();
	inputCorrespondencesWereLoaded = true;
	}

#define PROCESS_AND_MEASURE_TIME(dfn) \
	{ \
	beginTime = clock(); \
	dfn->process(); \
	endTime = clock(); \
	processingTime += float(endTime - beginTime) / CLOCKS_PER_SEC; \
	}

void ReconstructionTester::ExecuteDfns()
	{
	ASSERT(dfnsWereLoaded && inputCorrespondencesWereLoaded, "Cannot execute DFNs before loading inputs and dfns");
	clock_t beginTime, endTime;
	float processingTime = 0;

	fundamentalMatrixEstimator->matchesInput(*inputCorrespondenceMap);
	PROCESS_AND_MEASURE_TIME(fundamentalMatrixEstimator)
	DELETE_IF_NOT_NULL(fundamentalMatrix);
	Matrix3dPtr newFundamentalMatrix = NewMatrix3d();
	Copy( fundamentalMatrixEstimator->fundamentalMatrixOutput(), *newFundamentalMatrix);
	fundamentalMatrix = newFundamentalMatrix;
	fundamentalMatrixSuccess = fundamentalMatrixEstimator->successOutput();

	if (!fundamentalMatrixSuccess)
		{
		dfnsWereExecuted = true;
		return;
		}

	poseEstimator->matchesInput(*inputCorrespondenceMap);
	poseEstimator->fundamentalMatrixInput(*fundamentalMatrix);
	PROCESS_AND_MEASURE_TIME(poseEstimator)
	DELETE_IF_NOT_NULL(cameraTransform);
	Pose3DPtr newCameraTransform = NewPose3D();
	Copy( poseEstimator->transformOutput(), *newCameraTransform);
	cameraTransform = newCameraTransform;
	poseEstimatorWasSuccessful = poseEstimator->successOutput();

	if (!poseEstimatorWasSuccessful)
		{
		dfnsWereExecuted = true;
		return;
		}

	reconstructor->matchesInput(*inputCorrespondenceMap);
	reconstructor->poseInput(*cameraTransform);
	PROCESS_AND_MEASURE_TIME(reconstructor)
	DELETE_IF_NOT_NULL(outputPointCloud);
	PointCloudPtr newOutputPointCloud = NewPointCloud();
	Copy( reconstructor->pointcloudOutput(), *newOutputPointCloud);
	outputPointCloud = newOutputPointCloud;	
	dfnsWereExecuted = true;

	PRINT_TO_LOG("Processing took (seconds): ", processingTime);
	}

bool ReconstructionTester::AreTriangulatedPointsValid(float fieldOfViewX, float fieldOfViewY)
	{
	ASSERT(dfnsWereExecuted, "Error: cannot validate results without executing dfns first");

	if (!fundamentalMatrixSuccess)
		{
		PRINT_TO_LOG("Fundamental Matrix computation failed, it is impossible to triangulate the points", "");
		return false;
		}

	if (!poseEstimatorWasSuccessful)
		{
		PRINT_TO_LOG("Pose Estimation failed, it is impossible to triangulate the points", "");
		return false;
		}

	bool pointsAreValid = AllPointsAreInTheFieldOfView(fieldOfViewX, fieldOfViewY);
	
	if (!pointsAreValid)
		{
		PRINT_TO_LOG("Some triangulated points lay outside the field of view cone of the camera", "");
		}

	return pointsAreValid;
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void ReconstructionTester::LoadInputCorrespondences()
	{
	cv::FileStorage opencvFile(inputCorrespodencesFilePath, cv::FileStorage::READ);

	cv::Mat cvCorrespondenceMap;
	opencvFile["CorrespondenceMap"] >> cvCorrespondenceMap;
	opencvFile.release();

	ASSERT(cvCorrespondenceMap.rows > 0, "Error: correspondence map contains no keypoints");
	ASSERT(cvCorrespondenceMap.cols == 4, "Error: correspondence map has invalid format. It should contain a row for each correspondence, and four integers representing the image coordinates");
	ASSERT(cvCorrespondenceMap.type() == CV_16UC1, "Error: correspondence map invalid format. It should contain a row for each correspondence, and four integers representing the image coordinates");

	CorrespondenceMap2DPtr newReferenceCorrespondenceMap = NewCorrespondenceMap2D();

	for(unsigned row = 0; row < cvCorrespondenceMap.rows; row++)
		{
		BaseTypesWrapper::Point2D source, sink;
		source.x = cvCorrespondenceMap.at<uint16_t>(row, 0);
		source.y = cvCorrespondenceMap.at<uint16_t>(row, 1);
		sink.x = cvCorrespondenceMap.at<uint16_t>(row, 2);
		sink.y = cvCorrespondenceMap.at<uint16_t>(row, 3);

		AddCorrespondence(*newReferenceCorrespondenceMap, source, sink, 1);
		}

	DELETE_IF_NOT_NULL(inputCorrespondenceMap);
	inputCorrespondenceMap = newReferenceCorrespondenceMap;
	}

void ReconstructionTester::ConfigureDfns()
	{
	fundamentalMatrixEstimator->setConfigurationFile(fundamentalMatrixEstimatorFilePath);
	fundamentalMatrixEstimator->configure();

	poseEstimator->setConfigurationFile(poseEstimatorConfigurationFilePath);
	poseEstimator->configure();

	reconstructor->setConfigurationFile(reconstructorConfigurationFilePath);
	reconstructor->configure();

	dfnsWereLoaded = true;
	}

bool ReconstructionTester::AllPointsAreInTheFieldOfView(float fieldOfViewX, float fieldOfViewY)
	{
	int invalidPointsCounter = 0;
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*outputPointCloud); pointIndex++)
		{
		float x = GetXCoordinate(*outputPointCloud, pointIndex);
		float y = GetYCoordinate(*outputPointCloud, pointIndex);
		float z = GetZCoordinate(*outputPointCloud, pointIndex);

		float horizontalAngle = std::atan2(x, z);
		float verticalAngle = std::atan2(y, z);
		
		if ( std::abs(horizontalAngle) > fieldOfViewX || std::abs(verticalAngle) > fieldOfViewY )
			{
			std::stringstream pointStream;
			pointStream << "Point (" << x << ", " << y << ", " << z <<") is out of the field of view with the following angles (" << horizontalAngle << ", " << verticalAngle <<")";
			PRINT_TO_LOG(pointStream.str(), "");
			invalidPointsCounter++;
			}
		}

	PRINT_TO_LOG("Percentage of invalid points is:", ((float)invalidPointsCounter/(float)GetNumberOfPoints(*outputPointCloud)) );
	return (invalidPointsCounter == 0);
	}

/** @} */
