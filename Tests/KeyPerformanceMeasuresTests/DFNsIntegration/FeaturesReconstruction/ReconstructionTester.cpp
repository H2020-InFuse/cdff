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

using namespace dfn_ci;
using namespace Converters;
using namespace PointCloudWrapper;
using namespace MatrixWrapper;
using namespace PoseWrapper;
using namespace Common;
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

	SetUpMocksAndStubs();
	}

ReconstructionTester::~ReconstructionTester()
	{
	delete(stubTransformCache);
	delete(mockTransformConverter);

	delete(stubInverseTransformCache);
	delete(mockInverseTransformConverter);
	
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

void ReconstructionTester::ExecuteDfns()
	{
	ASSERT(dfnsWereLoaded && inputCorrespondencesWereLoaded, "Cannot execute DFNs before loading inputs and dfns");

	fundamentalMatrixEstimator->correspondenceMapInput(inputCorrespondenceMap);
	fundamentalMatrixEstimator->process();
	DELETE_IF_NOT_NULL(fundamentalMatrix);
	fundamentalMatrix = fundamentalMatrixEstimator->fundamentalMatrixOutput();
	fundamentalMatrixSuccess = fundamentalMatrixEstimator->successOutput();

	if (!fundamentalMatrixSuccess)
		{
		dfnsWereExecuted = true;
		return;
		}

	poseEstimator->correspondenceMapInput(inputCorrespondenceMap);
	poseEstimator->fundamentalMatrixInput(fundamentalMatrix);
	poseEstimator->process();
	DELETE_IF_NOT_NULL(cameraTransform);
	cameraTransform = poseEstimator->transformOutput();
	poseEstimatorWasSuccessful = poseEstimator->successOutput();

	if (!poseEstimatorWasSuccessful)
		{
		dfnsWereExecuted = true;
		return;
		}

	reconstructor->correspondenceMapInput(inputCorrespondenceMap);
	reconstructor->poseInput(cameraTransform);
	reconstructor->process();
	DELETE_IF_NOT_NULL(outputPointCloud);
	outputPointCloud = reconstructor->pointCloudOutput();	
	dfnsWereExecuted = true;
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
void ReconstructionTester::SetUpMocksAndStubs()
	{
	stubTransformCache = new Stubs::CacheHandler<cv::Mat, Transform3DConstPtr>;
	mockTransformConverter = new Mocks::MatToTransform3DConverter();
	ConversionCache<cv::Mat, Transform3DConstPtr, MatToTransform3DConverter>::Instance(stubTransformCache, mockTransformConverter);

	stubInverseTransformCache = new Stubs::CacheHandler<Transform3DConstPtr, cv::Mat>;
	mockInverseTransformConverter = new Mocks::Transform3DToMatConverter();
	ConversionCache<Transform3DConstPtr, cv::Mat, Transform3DToMatConverter>::Instance(stubInverseTransformCache, mockInverseTransformConverter);
	}

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
