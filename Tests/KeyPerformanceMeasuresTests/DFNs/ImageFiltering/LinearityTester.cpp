/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file LinearityTester.cpp
 * @date 09/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 *
 * Implementation of the LinearityTester class.
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
#include "LinearityTester.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ctime>

using namespace CDFF::DFN;
using namespace Converters;
using namespace FrameWrapper;

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
LinearityTester::LinearityTester()
	{
	dfn = NULL;
	inputFrame = NULL;
	outputFrame = NULL;

	dfnWasSet = false;
	inputImageWasLoaded = false;
	}

LinearityTester::~LinearityTester()
	{
	DELETE_IF_NOT_NULL(inputFrame);
	DELETE_IF_NOT_NULL(outputFrame);
	}

void LinearityTester::SetDfn(const std::string& configurationFilePath, CDFF::DFN::ImageFilteringInterface* dfn)
	{
	this->configurationFilePath = configurationFilePath;
	this->dfn = dfn;
	ConfigureDfn();

	dfnWasSet = true;
	}

void LinearityTester::SetFilesPaths(const std::string& inputImageFilePath, const std::string& outputImageFilePath)
	{
	this->inputImageFilePath = inputImageFilePath;
	this->outputImageFilePath = outputImageFilePath;

	LoadInputImage();
	inputImageWasLoaded = true;
	}

void LinearityTester::ExecuteDfn()
	{
	ASSERT(dfnWasSet && inputImageWasLoaded, "Error: dfn or inputs were not correctly loaded");

	dfn->imageInput(*inputFrame);

	clock_t beginTime = clock();
	dfn->process();
	clock_t endTime = clock();
	float processingTime = float(endTime - beginTime) / CLOCKS_PER_SEC;
	PRINT_TO_LOG("Processing took (seconds): ", processingTime);

	DELETE_IF_NOT_NULL(outputFrame);
	FramePtr newOutputFrame = NewFrame();
	Copy( dfn->imageOutput(), *newOutputFrame);
	outputFrame = newOutputFrame;
	}

bool LinearityTester::IsResultLinear(const std::string& referenceLinesFilePath, float relativeDistortionDifference)
	{
	this->referenceLinesFilePath = referenceLinesFilePath;
	LoadReferenceLines();

	float totalError, averageError, averageRelativeError;
	ComputeErrors(totalError, averageError, averageRelativeError);

	bool averageRelativeErrorIsBelowThreshold = (averageRelativeError <= relativeDistortionDifference);
	if (!averageRelativeErrorIsBelowThreshold)
		{
		PRINT_TO_LOG("Average relative error exceeds the defined threshold:", relativeDistortionDifference);
		}
	return averageRelativeErrorIsBelowThreshold;
	}

void LinearityTester::SaveOutputImage()
	{
	outputImage = inverseFrameConverter.Convert(outputFrame);
	cv::imwrite(outputImageFilePath, outputImage);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void LinearityTester::LoadInputImage()
	{
	inputImage = cv::imread(inputImageFilePath, CV_LOAD_IMAGE_COLOR);
	ASSERT(inputImage.cols > 0 && inputImage.rows > 0, "Error: Loaded input image is empty");

	DELETE_IF_NOT_NULL(inputFrame);
	inputFrame = frameConverter.Convert(inputImage);
	}

void LinearityTester::LoadReferenceLines()
	{
	cv::Mat linesMatrix;
	cv::FileStorage opencvFile(referenceLinesFilePath, cv::FileStorage::READ);
	opencvFile["LinesMatrix"] >> linesMatrix;
	opencvFile.release();
	ASSERT(linesMatrix.rows > 0 && linesMatrix.cols == 3 && linesMatrix.type() == CV_16UC1, "Error: reference lines are invalid");

	double minLineIndex, maxLineIndex;
	cv::minMaxLoc(linesMatrix(cv::Rect(2, 0, 1, linesMatrix.rows)), &minLineIndex, &maxLineIndex);
	ASSERT(minLineIndex == 0, "Error, output xml file contains some data but the format is incorrect");


	for(int lineIndex = 0; lineIndex <= maxLineIndex; lineIndex++)
		{
		Line newLine;
		referenceLinesList.push_back(newLine);
		}

	for(int pointIndex = 0; pointIndex < linesMatrix.rows; pointIndex++)
		{
		uint16_t lineIndex = linesMatrix.at<uint16_t>(pointIndex, 2);
		Point newPoint;
		newPoint.x = linesMatrix.at<uint16_t>(pointIndex, 0);
		newPoint.y = linesMatrix.at<uint16_t>(pointIndex, 1);
		referenceLinesList.at(lineIndex).push_back(newPoint);
		}
	}

void LinearityTester::ConfigureDfn()
	{
	dfn->setConfigurationFile(configurationFilePath);
	dfn->configure();
	}

void LinearityTester::ComputeErrors(float& totalError, float& averageError, float& averageRelativeError)
	{
	float totalRelativeError = 0;
	totalError = 0;
	for(int lineIndex = 0; lineIndex < referenceLinesList.size(); lineIndex++)
		{
		float error = ComputeErrorOnLine(referenceLinesList.at(lineIndex));
		errorsList.push_back(error);
		totalError += error;

		float lineLength = ComputeLineLength(referenceLinesList.at(lineIndex));
		float relativeError = error / lineLength;
		relativeErrorsList.push_back(relativeError);
		totalRelativeError += relativeError;
		}

	averageError = totalError/static_cast<float>(referenceLinesList.size());
	averageRelativeError = totalRelativeError/static_cast<float>(referenceLinesList.size());

	PRINT_TO_LOG("Total error:", totalError);
	PRINT_TO_LOG("Average error:", averageError );
	PRINT_TO_LOG("Average relative error", averageRelativeError );
	}

float LinearityTester::ComputeErrorOnLine(const Line& line)
	{
	cv::Mat pointsMatrix( line.size(), 2, CV_32FC1);
	for(int pointIndex = 0; pointIndex < line.size(); pointIndex++)
		{
		pointsMatrix.at<float>(pointIndex, 0) = line.at(pointIndex).x;
		pointsMatrix.at<float>(pointIndex, 1) = line.at(pointIndex).y;
		}

	cv::Vec4f lineModel; // (lineModel[0], lineModel[1]) is the line vector, (lineModel[2], lineModel[3]) is the line origin
	cv::fitLine(pointsMatrix, lineModel, CV_DIST_L1, 0, 0.01, 0.01);

	float totalError = 0;
	for(int pointIndex = 0; pointIndex < line.size(); pointIndex++)
		{
		float differenceX = line.at(pointIndex).x - lineModel[2];
		float differenceY = line.at(pointIndex).y - lineModel[3];
		float lineOriginToPointDistance = std::sqrt(differenceX*differenceX + differenceY*differenceY);
		float lineVectorProductOriginToPointVector = lineModel[0]*differenceX + lineModel[1]*differenceY;
		float lineToPointDistance = std::abs(lineVectorProductOriginToPointVector / lineOriginToPointDistance);
		totalError += lineToPointDistance;
		}

	return totalError;
	}

float LinearityTester::ComputeLineLength(const Line& line)
	{
	float maxDistance = 0;
	for(int point1Index = 0; point1Index < line.size(); point1Index++)
		{
		const Point& point1 = line.at(point1Index);
		for (int point2Index = point1Index+1; point2Index < line.size(); point2Index++)
			{
			const Point& point2 = line.at(point2Index);
			float distance = std::sqrt( (point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y) );
			if (distance > maxDistance)
				{
				maxDistance = distance;
				}
			}
		}
	return maxDistance;
	}

/** @} */
