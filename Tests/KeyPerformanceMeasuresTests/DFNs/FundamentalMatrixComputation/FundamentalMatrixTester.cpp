/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FundamentalMatrixTester.cpp
 * @date 15/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the FundamentalMatrixTester class.
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
#include "FundamentalMatrixTester.hpp"
#include <ctime>


using namespace dfn_ci;
using namespace MatrixWrapper;
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
FundamentalMatrixTester::FundamentalMatrixTester(std::string configurationFilePath, dfn_ci::FundamentalMatrixComputationInterface* dfn) 
	{
	this->configurationFilePath = configurationFilePath;
	this->dfn = dfn;

	inputCorrespondenceMap = NULL;
	outputFundamentalMatrix = NULL;

	inputCorrespondencesWereLoaded = false;

	ConfigureDfn();
	}

FundamentalMatrixTester::~FundamentalMatrixTester()
	{
	DELETE_IF_NOT_NULL(inputCorrespondenceMap);
	DELETE_IF_NOT_NULL(outputFundamentalMatrix);
	}

void FundamentalMatrixTester::SetInputFilePath(std::string inputCorrespondenceFilePath)
	{
	this->inputCorrespondenceFilePath = inputCorrespondenceFilePath;

	LoadInputCorrespondences();
	inputCorrespondencesWereLoaded = true;
	}

void FundamentalMatrixTester::ExecuteDfn()
	{
	dfn->correspondenceMapInput(inputCorrespondenceMap);

	clock_t beginTime = clock();
	dfn->process();
	clock_t endTime = clock();
	float processingTime = float(endTime - beginTime) / CLOCKS_PER_SEC;
	PRINT_TO_LOG("Processing took (seconds): ", processingTime);

	DELETE_IF_NOT_NULL(outputFundamentalMatrix);
	outputFundamentalMatrix = dfn->fundamentalMatrixOutput();
	outputComputationSuccess = dfn->successOutput();
	}

bool FundamentalMatrixTester::IsDfnSuccessful()
	{
	PRINT_TO_LOG("The number of matches is:", GetNumberOfCorrespondences(*inputCorrespondenceMap));

	PrintOutputInformation();

	return outputComputationSuccess;
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void FundamentalMatrixTester::LoadInputCorrespondences()
	{
	cv::FileStorage opencvFile(inputCorrespondenceFilePath, cv::FileStorage::READ);

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

	inputCorrespondenceMap = newReferenceCorrespondenceMap;
	}

void FundamentalMatrixTester::ConfigureDfn()
	{
	dfn->setConfigurationFile(configurationFilePath);
	dfn->configure();
	}

void FundamentalMatrixTester::PrintOutputInformation()
	{
	if (!outputComputationSuccess)
		{
		PRINT_TO_LOG("We could not compute the fundamental matrix", "");
		return;
		}
	else
		{
		PRINT_TO_LOG("The fundamental matrix was computed succesfully", "");
		}

	cv::Mat cvFundamentalMatrix(3, 3, CV_32FC1);
	for(int row = 0; row < 3; row++)
		{
		for (int column = 0; column < 3; column++)
			{
			cvFundamentalMatrix.at<float>(row,column) = GetElement(*outputFundamentalMatrix, row, column);
			}
		}
	PRINT_TO_LOG("The fundamental matrix is: \n", cvFundamentalMatrix);

	float error = ComputeError(cvFundamentalMatrix);
	PRINT_TO_LOG("Matrix error: ", error);
	}

float FundamentalMatrixTester::ComputeError(cv::Mat cvFundamentalMatrix)
	{
	float totalError = 0;
	for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*inputCorrespondenceMap); correspondenceIndex++)
		{
		BaseTypesWrapper::Point2D source = GetSource(*inputCorrespondenceMap, correspondenceIndex);
		BaseTypesWrapper::Point2D sink = GetSink(*inputCorrespondenceMap, correspondenceIndex);

		cv::Mat sourceColumn(3, 1, CV_32FC1);
		sourceColumn.at<float>(0,0) = source.x;
		sourceColumn.at<float>(1,0) = source.y;
		sourceColumn.at<float>(2,0) = 1.0;

		cv::Mat sinkRow(1, 3, CV_32FC1);
		sinkRow.at<float>(0,0) = sink.x;
		sinkRow.at<float>(0,1) = sink.y;
		sinkRow.at<float>(0,2) = 1.0;

		cv::Mat error = sinkRow * cvFundamentalMatrix * sourceColumn;
		ASSERT(error.rows == 1 && error.cols == 1, "Error does not have the expected format");
		totalError += std::abs( error.at<float>(0,0) );
		}

	return (totalError / (float)GetNumberOfCorrespondences(*inputCorrespondenceMap) );
	}

/** @} */
