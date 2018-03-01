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

#include <ConversionCache/ConversionCache.hpp>
#include <Converters/FrameToMatConverter.hpp>
#include <BaseTypes.hpp>

using namespace FrameWrapper;
using namespace Converters;
using namespace Common;
using namespace VisualPointFeatureVector2DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace MatrixWrapper;

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

	cv::Mat image = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(frame);
	ShowImage(image);
	}

void OpencvVisualizer::ShowVisualFeatures(FrameConstPtr frame, VisualPointFeatureVector2DConstPtr featuresVector)
	{
	RETURN_IF_DISABLED
	cv::Mat image = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(frame);

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
	cv::Mat sourceImage = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(frame1);
	cv::Mat sinkImage = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(frame2);

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

}
/** @} */

