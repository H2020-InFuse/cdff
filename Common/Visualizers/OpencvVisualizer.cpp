/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file OpencvVisualizer.cpp
 * @date 28/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Visualizers
 * 
 * Implementation of the OpencvVisualizer class
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
#include "OpencvVisualizer.hpp"
#include <Errors/Assert.hpp>
#include <BaseTypes.hpp>

using namespace FrameWrapper;
using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace MatrixWrapper;
using namespace BaseTypesWrapper;

#define RETURN_IF_DISABLED \
	if (!enabled) \
		{ \
		return; \
		}

namespace Visualizers
{
/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
void OpencvVisualizer::ShowImage(cv::Mat image)
	{
	RETURN_IF_DISABLED

	cv::imshow(WINDOW_NAME, image);
	cv::waitKey();
	}

void OpencvVisualizer::ShowImage(FrameConstPtr frame)
	{
	RETURN_IF_DISABLED

	cv::Mat image = converter.Convert(frame);
	ShowImage(image);
	}

void OpencvVisualizer::ShowVisualFeatures(FrameConstPtr frame, VisualPointFeatureVector2DConstPtr featuresVector)
	{
	RETURN_IF_DISABLED
	cv::Mat image = converter.Convert(frame);

	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*featuresVector); pointIndex++)
		{
		cv::Point drawPoint(GetXCoordinate(*featuresVector, pointIndex), GetYCoordinate(*featuresVector, pointIndex) );
		cv::circle(image, drawPoint, 5, cv::Scalar(100, 0, 0), 2, 8, 0);
		}
	ShowImage(image);
	}

void OpencvVisualizer::ShowCorrespondences(FrameConstPtr frame1, FrameConstPtr frame2, CorrespondenceMap2DConstPtr correspondenceMap)
	{
	RETURN_IF_DISABLED
	cv::Mat sourceImage = converter.Convert(frame1);
	cv::Mat sinkImage = converter.Convert(frame2);

	std::vector<cv::KeyPoint> sourceVector, sinkVector;
	std::vector<cv::DMatch> matchesVector;
	for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMap); correspondenceIndex++)
		{
		BaseTypesWrapper::Point2D sourcePoint, sinkPoint;
		sourcePoint = GetSource(*correspondenceMap, correspondenceIndex);
		sinkPoint = GetSink(*correspondenceMap, correspondenceIndex);
		cv::KeyPoint sourceKeypoint(sourcePoint.x, sourcePoint.y, 0.02);
		cv::KeyPoint sinkKeypoint(sinkPoint.x, sinkPoint.y, 0.02);
		sourceVector.push_back(sourceKeypoint);
		sinkVector.push_back(sinkKeypoint);
		cv::DMatch match;
		match.queryIdx = correspondenceIndex;
		match.trainIdx = correspondenceIndex;
		matchesVector.push_back(match);
		}

	cv::Mat outputImage;
 	cv::drawMatches
		(
		sourceImage, sourceVector, sinkImage, sinkVector, matchesVector, 
		outputImage, 
		cv::Scalar::all(-1), cv::Scalar::all(-1),
               	std::vector<char>(), 
		cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
		);
	ShowImage(outputImage);
	}

#define CONTINUE_ON_INVALID_2D_POINT(point) \
	if (point.x != point.x || point.y != point.y) \
		{ \
		continue; \
		}
#define CONTINUE_ON_DISTINCT_POINTS(point1, point2) \
	if (point1.x != point2.x || point1.y != point2.y) \
		{ \
		continue; \
		}

void OpencvVisualizer::ShowQuadrupleCorrespondences(std::vector<FrameConstPtr> frameList, std::vector<CorrespondenceMap2DConstPtr> correspondenceMapList)
	{
	RETURN_IF_DISABLED
	ASSERT(frameList.size() == 4 && correspondenceMapList.size() == 4, "Opencv Visualizer, ShowQuadrupleCorrespondences, Both inputs should have size 4");
	int rows = 0;
	int cols = 0;

	cv::Mat imageList[4];
	for(int imageIndex = 0; imageIndex < 4; imageIndex++)
		{
		imageList[imageIndex] = converter.Convert(frameList.at(imageIndex));
		if (imageIndex == 0)
			{
			rows = imageList[imageIndex].rows;
			cols = imageList[imageIndex].cols;
			}
		else
			{
			ASSERT( rows == imageList[imageIndex].rows && cols == imageList[imageIndex].cols, "Image dimensions do not match" );
			}
		}

	cv::Mat temporaryList1[2] = { imageList[0], imageList[1] };
	cv::Mat temporaryList2[2] = { imageList[2], imageList[3] };
	cv::Mat outputImage1, outputImage2, outputImage3;
	cv::hconcat(temporaryList1, 2, outputImage1);
	cv::hconcat(temporaryList2, 2, outputImage2);
	cv::Mat temporaryList3[2] = { outputImage1, outputImage2 };
	cv::vconcat(temporaryList3, 2, outputImage3);
	
	int count = 0;
	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = correspondenceMapList.at(0);
	for(int correspondenceIndex1 = 0; correspondenceIndex1 < GetNumberOfCorrespondences(*leftRightCorrespondenceMap); correspondenceIndex1++)
		{
		Point2D leftRightSource = GetSource(*leftRightCorrespondenceMap, correspondenceIndex1);
		Point2D leftRightSink = GetSink(*leftRightCorrespondenceMap, correspondenceIndex1);
		CONTINUE_ON_INVALID_2D_POINT(leftRightSource);
		CONTINUE_ON_INVALID_2D_POINT(leftRightSink);
		CorrespondenceMap2DConstPtr leftTimeCorrespondenceMap = correspondenceMapList.at(1);
		for(int correspondenceIndex2 = 0; correspondenceIndex2 < GetNumberOfCorrespondences(*leftTimeCorrespondenceMap); correspondenceIndex2++)
			{
			Point2D leftTimeSource = GetSource(*leftTimeCorrespondenceMap, correspondenceIndex2);
			Point2D leftTimeSink = GetSink(*leftTimeCorrespondenceMap, correspondenceIndex2);
			CONTINUE_ON_INVALID_2D_POINT(leftTimeSource);
			CONTINUE_ON_INVALID_2D_POINT(leftTimeSink);
			CONTINUE_ON_DISTINCT_POINTS(leftRightSource, leftTimeSource);
			CorrespondenceMap2DConstPtr rightTimeCorrespondenceMap = correspondenceMapList.at(2);
			for(int correspondenceIndex3 = 0; correspondenceIndex3 < GetNumberOfCorrespondences(*rightTimeCorrespondenceMap); correspondenceIndex3++)
				{
				Point2D rightTimeSource = GetSource(*rightTimeCorrespondenceMap, correspondenceIndex3);
				Point2D rightTimeSink = GetSink(*rightTimeCorrespondenceMap, correspondenceIndex3);
				CONTINUE_ON_INVALID_2D_POINT(rightTimeSource);
				CONTINUE_ON_INVALID_2D_POINT(rightTimeSink);
				CONTINUE_ON_DISTINCT_POINTS(leftRightSink, rightTimeSource);
				CorrespondenceMap2DConstPtr pastCorrespondenceMap = correspondenceMapList.at(3);
				for(int correspondenceIndex4 = 0; correspondenceIndex4 < GetNumberOfCorrespondences(*pastCorrespondenceMap); correspondenceIndex4++)
					{
					Point2D pastSource = GetSource(*pastCorrespondenceMap, correspondenceIndex4);
					Point2D pastSink = GetSink(*pastCorrespondenceMap, correspondenceIndex4);
					CONTINUE_ON_INVALID_2D_POINT(pastSource);
					CONTINUE_ON_INVALID_2D_POINT(pastSink);
					CONTINUE_ON_DISTINCT_POINTS(pastSource, leftTimeSink);
					CONTINUE_ON_DISTINCT_POINTS(pastSink, rightTimeSink);

					count++;
					cv::Scalar color( (50 + count * 100) % 255, (50 + count * 10) % 255, (50 + count * 1000) % 255);
					cv::line(outputImage3, cv::Point(leftRightSource.x, leftRightSource.y), cv::Point(leftRightSink.x + cols, leftRightSink.y), color);
					cv::line(outputImage3, cv::Point(leftTimeSource.x, leftTimeSource.y), cv::Point(leftTimeSink.x, leftTimeSink.y + rows), color); 
					cv::line(outputImage3, cv::Point(rightTimeSource.x + cols, rightTimeSource.y), cv::Point(rightTimeSink.x + cols, rightTimeSink.y + rows), color); 
					cv::line(outputImage3, cv::Point(pastSource.x, pastSource.y + rows), cv::Point(pastSink.x + cols, pastSink.y + rows), color); 
					}
				}
			}
		}

	ShowImage(outputImage3);
	}

void OpencvVisualizer::ShowDisparity(cv::Mat disparity)
	{
	RETURN_IF_DISABLED
	cv::Mat normalizedDisparity;
	cv::normalize(disparity, normalizedDisparity, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	ShowImage(normalizedDisparity);
	}

void OpencvVisualizer::ShowMatrix(Matrix3dConstPtr matrix)
	{
	RETURN_IF_DISABLED
	std::stringstream stream;
	stream << "\n" << GetElement(*matrix, 0, 0) <<", "<< GetElement(*matrix, 0, 1) <<", "<< GetElement(*matrix, 0, 2);
	stream << "\n" << GetElement(*matrix, 1, 0) <<", "<< GetElement(*matrix, 1, 1) <<", "<< GetElement(*matrix, 1, 2);
	stream << "\n" << GetElement(*matrix, 2, 0) <<", "<< GetElement(*matrix, 2, 1) <<", "<< GetElement(*matrix, 2, 2);
	std::string matrixString = stream.str();
	PRINT_TO_LOG("Matrix: ", matrixString);
	}

void OpencvVisualizer::ShowPose(Pose3DConstPtr pose)
	{
	RETURN_IF_DISABLED
	std::stringstream stream;
	stream << "Position: (" << GetXPosition(*pose) <<", "<< GetYPosition(*pose) <<", "<< GetZPosition(*pose) <<"); ";
	stream << "Orientation: (" << GetXRotation(*pose) <<", "<< GetYRotation(*pose) <<", "<< GetZRotation(*pose) <<", "<< GetWRotation(*pose) <<"); ";
	std::string poseString = stream.str();
	PRINT_TO_LOG("Pose: ", poseString);
	}

void OpencvVisualizer::Enable()
	{
	enabled = true;
	}

void OpencvVisualizer::Disable()
	{
	enabled = false;
	}

/* --------------------------------------------------------------------------
 *
 * Protected Member Functions
 *
 * --------------------------------------------------------------------------
 */
OpencvVisualizer::OpencvVisualizer()
	{

	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const std::string OpencvVisualizer::WINDOW_NAME = "Opencv Visualizer";
bool OpencvVisualizer::enabled = false;
FrameToMatConverter OpencvVisualizer::converter;

}
/** @} */

