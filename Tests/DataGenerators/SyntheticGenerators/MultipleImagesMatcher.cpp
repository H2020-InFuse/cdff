/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MultipleImagesMatcher.cpp
 * @date 19/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Implementation of the MultipleImagesMatcher class.
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
#include "MultipleImagesMatcher.hpp"
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <Errors/Assert.hpp>
#include <Converters/MatToCorrespondenceMaps2DSequenceConverter.hpp>
#include <Converters/CorrespondenceMaps2DSequenceToMatConverter.hpp>

#define DELETE_IF_NOT_NULL(pointer) \
	if (pointer != NULL) \
		{ \
		delete(pointer); \
		}

#define PRINT_SOURCE_SINK_INFO \
	{ \
	std::stringstream sinkSourceStream; \
	sinkSourceStream << "Source: " << currentSourceIndex << "  Sink: " << currentSinkIndex; \
	PRINT_TO_LOG(sinkSourceStream.str(), ""); \
	}

#define POSSIBLE_UNORDERED_PAIRS(vector) (vector.size() * (vector.size()-1) / 2)	
	
using namespace Converters;
using namespace CorrespondenceMap2DWrapper;

namespace DataGenerators
{

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
MultipleImagesMatcher::MultipleImagesMatcher(std::vector<std::string> inputImageFilePathList, std::string outputCorrespondencesFilePath) :
	imageZoomingList( inputImageFilePathList.size() ),
	originalImageList( inputImageFilePathList.size() ),
	selectionList( inputImageFilePathList.size() ),
	correspondenceVectorList( POSSIBLE_UNORDERED_PAIRS(inputImageFilePathList) ),
	inputImageFilePathList(inputImageFilePathList)
{
	this->outputCorrespondencesFilePath = outputCorrespondencesFilePath;
	
	for(int imageIndex = 0; imageIndex < inputImageFilePathList.size(); imageIndex++)
		{
		imageZoomingList.at(imageIndex) = NULL;
		}
	currentSourceIndex = 0;
	currentSinkIndex = 1;
	PRINT_SOURCE_SINK_INFO;

	LoadImages();
	LoadCorrespondences();

	cv::namedWindow("Images Matcher", 1);
	cv::setMouseCallback("Images Matcher", MultipleImagesMatcher::MouseCallback, this);
	}

MultipleImagesMatcher::~MultipleImagesMatcher()
	{
	for(int imageIndex = 0; imageIndex < inputImageFilePathList.size(); imageIndex++)
		{
		DELETE_IF_NOT_NULL( imageZoomingList.at(imageIndex) );
		}
	}

void MultipleImagesMatcher::Run()
	{
	for(int imageIndex = 0; imageIndex < inputImageFilePathList.size(); imageIndex++)
		{
		ASSERT(originalImageList.at(imageIndex).rows > 0 && originalImageList.at(imageIndex).cols >0, "Images Matcher error, an input image is empty");
		}

	bool painterIsActive = true;
	while(painterIsActive)
		{
		DrawImages();
		char command = cv::waitKey(30);
		if (command == 'q' || command == 'Q')
			{
			painterIsActive = false;
			}
		else
			{
			ExecuteCommand(command);
			}
		}
	}


/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const int MultipleImagesMatcher::BASE_WINDOW_WIDTH = 800;
const int MultipleImagesMatcher::BASE_WINDOW_HEIGHT = 600;
const std::vector<cv::Scalar> MultipleImagesMatcher::COLORS_LIST =
	{
	cv::Scalar(0, 0, 255),
	cv::Scalar(0, 255, 0),
	cv::Scalar(255, 0, 0),
	cv::Scalar(0, 255, 255),
	cv::Scalar(255, 255, 0),
	cv::Scalar(255, 0, 255),
	cv::Scalar(128, 128, 255),
	cv::Scalar(128, 255, 128),
	cv::Scalar(255, 128, 128),
	cv::Scalar(128, 255, 255),
	cv::Scalar(255, 255, 128),
	cv::Scalar(255, 128, 255),
	cv::Scalar(0, 0, 128),
	cv::Scalar(0, 128, 0),
	cv::Scalar(128, 0, 0),
	cv::Scalar(0, 128, 128),
	cv::Scalar(128, 128, 0),
	cv::Scalar(128, 0, 128)
	};


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void MultipleImagesMatcher::LoadImages()
	{
	for(int imageIndex = 0; imageIndex < inputImageFilePathList.size(); imageIndex++)
		{
		originalImageList.at(imageIndex) = cv::imread(inputImageFilePathList.at(imageIndex), CV_LOAD_IMAGE_COLOR);
		}

	for(int imageIndex = 0; imageIndex < inputImageFilePathList.size(); imageIndex++)
		{
		DELETE_IF_NOT_NULL( imageZoomingList.at(imageIndex) );
		imageZoomingList.at(imageIndex) = new ImageZooming(originalImageList.at(imageIndex).cols, originalImageList.at(imageIndex).rows, BASE_WINDOW_WIDTH, BASE_WINDOW_HEIGHT);
		}
	}

void MultipleImagesMatcher::MouseCallback(int event, int x, int y, int flags, void* userdata)
	{
	((MultipleImagesMatcher*)userdata)->MouseCallback(event, x, y); 
	}

void MultipleImagesMatcher::MouseCallback(int event, int x, int y)
	{
	if (event != cv::EVENT_LBUTTONDOWN && event != cv::EVENT_RBUTTONDOWN)
		{
		return;
		}

	std::vector<Correspondence>& correspondencesVector = GetCurrentCorrespondenceVector();
	//In case of right mouse key-press we delete the last entry from the correspondencesVector: i.e. either the last selected keypoint or the last correspondence is deleted.
	if (event == cv::EVENT_RBUTTONDOWN)
		{
		if (correspondencesVector.size() > 0)
			{
			correspondencesVector.pop_back();
			}
		return;
		}

	//The type of selection depends on whether the x coordinate is on the left (source) or right (sink).
	SelectionType currentSelection = ( x < BASE_WINDOW_WIDTH ) ? SELECTED_FROM_SOURCE : SELECTED_FROM_SINK;

	//Computes the effective coordinates on the image (due to zoom level the window coordinates and the image coordinates are different).
	int effectiveX, effectiveY;
	bool validPoint;
	if (currentSelection == SELECTED_FROM_SOURCE)
		{
		validPoint = imageZoomingList.at(currentSourceIndex)->WindowToImagePixel(x, y, effectiveX, effectiveY);
		}
	else
		{
		validPoint = imageZoomingList.at(currentSinkIndex)->WindowToImagePixel(x - BASE_WINDOW_WIDTH, y, effectiveX, effectiveY);
		}
	if (!validPoint)
		{
		return;
		}
	if (currentSelection == SELECTED_FROM_SOURCE)
		{
		GetClosePoint(selectionList.at(currentSourceIndex), effectiveX, effectiveY);
		}
	else
		{
		GetClosePoint(selectionList.at(currentSinkIndex), effectiveX, effectiveY);
		}

	
	//Add one selected point at the bottom of the correspondencesVector in case the vector is empty or the last correspondence is complete.
	if (correspondencesVector.size() == 0 || (correspondencesVector.end()-1)->selection == SELECTED_FROM_BOTH)
		{
		Correspondence correspondence;
		if (currentSelection == SELECTED_FROM_SOURCE)
			{
			correspondence.sourceX = effectiveX;
			correspondence.sourceY = effectiveY;
			}
		else
			{
			correspondence.sinkX = effectiveX;
			correspondence.sinkY = effectiveY;
			}
		correspondence.selection = currentSelection;
		correspondencesVector.push_back(correspondence);
		return;
		}

	//If the last correspondence is not complete, we add the corresponding point to the previous selection.
	std::vector<Correspondence>::iterator lastEntry = correspondencesVector.end()-1;
	bool selectionComplete = false;
	if (lastEntry->selection == SELECTED_FROM_SOURCE && currentSelection == SELECTED_FROM_SINK)
		{
		lastEntry->sinkX = effectiveX;
		lastEntry->sinkY = effectiveY;
		lastEntry->selection = SELECTED_FROM_BOTH;
		selectionComplete = true;
		}
	else if (lastEntry->selection == SELECTED_FROM_SINK && currentSelection == SELECTED_FROM_SOURCE)
		{
		lastEntry->sourceX = effectiveX;
		lastEntry->sourceY = effectiveY;
		lastEntry->selection = SELECTED_FROM_BOTH;
		selectionComplete = true;
		}

	// Add points to selections list if the selection is complete.
	if (selectionComplete)
		{
		AddToSelectionList(lastEntry->sourceX, lastEntry->sourceY, selectionList.at(currentSourceIndex));
		AddToSelectionList(lastEntry->sinkX, lastEntry->sinkY, selectionList.at(currentSinkIndex));
		}
	}

void MultipleImagesMatcher::DrawImages()
	{
	cv::Mat sourceImageToDraw = imageZoomingList.at(currentSourceIndex)->ExtractZoomedWindow(originalImageList.at(currentSourceIndex));
	cv::Mat sinkImageToDraw = imageZoomingList.at(currentSinkIndex)->ExtractZoomedWindow(originalImageList.at(currentSinkIndex));
	cv::Mat imageToDraw;
	cv::hconcat(sourceImageToDraw, sinkImageToDraw, imageToDraw);

	DrawCorrespondences(imageToDraw);
	cv::imshow("Images Matcher", imageToDraw);
	}

void MultipleImagesMatcher::DrawCorrespondences(cv::Mat imageToDraw)
	{
	std::vector<Correspondence>& correspondencesVector = GetCurrentCorrespondenceVector();
	unsigned nextColorIndex = 0;
	for(unsigned index = 0; index < correspondencesVector.size() && correspondencesVector.at(index).selection == SELECTED_FROM_BOTH; index++)
		{
		Correspondence& correspondence = correspondencesVector.at(index);
		cv::Scalar nextColor = COLORS_LIST.at(nextColorIndex % COLORS_LIST.size());

		int windowSourceX, windowSourceY, windowSinkX, windowSinkY;
		bool sourcePointIsVisible = imageZoomingList.at(currentSourceIndex)->ImageToWindowPixel(correspondence.sourceX, correspondence.sourceY, windowSourceX, windowSourceY);
		bool sinkPointIsVisible = imageZoomingList.at(currentSinkIndex)->ImageToWindowPixel(correspondence.sinkX, correspondence.sinkY, windowSinkX, windowSinkY);

		if (sourcePointIsVisible && sinkPointIsVisible)
			{
			cv::line(imageToDraw, cv::Point2d(windowSourceX, windowSourceY), cv::Point2d(windowSinkX + BASE_WINDOW_WIDTH, windowSinkY), nextColor, 1, 8);
			nextColorIndex++;
			}
		}

	DrawLastSelectedPoint(imageToDraw, nextColorIndex);
	DrawSelections(imageToDraw);
	}

void MultipleImagesMatcher::DrawLastSelectedPoint(cv::Mat imageToDraw, int nextColorIndex)
	{
	std::vector<Correspondence>& correspondencesVector = GetCurrentCorrespondenceVector();
	if (correspondencesVector.size() == 0)
		{
		return;
		}

	std::vector<Correspondence>::iterator lastEntry = correspondencesVector.end()-1;
	cv::Scalar nextColor = COLORS_LIST.at( nextColorIndex % COLORS_LIST.size());
	if ( lastEntry->selection == SELECTED_FROM_SOURCE )
		{
		int windowSourceX, windowSourceY;
		bool sourcePointIsVisible = imageZoomingList.at(currentSourceIndex)->ImageToWindowPixel(lastEntry->sourceX, lastEntry->sourceY, windowSourceX, windowSourceY);
		if (sourcePointIsVisible)
			{
			cv::circle(imageToDraw, cv::Point2d(windowSourceX, windowSourceY), 3, nextColor, -1);
			}
		}
	else if ( lastEntry->selection == SELECTED_FROM_SINK )
		{
		int windowSinkX, windowSinkY;
		bool sinkPointIsVisible = imageZoomingList.at(currentSinkIndex)->ImageToWindowPixel(lastEntry->sinkX, lastEntry->sinkY, windowSinkX, windowSinkY);
		if (sinkPointIsVisible)
			{
			cv::circle(imageToDraw, cv::Point2d(windowSinkX + BASE_WINDOW_WIDTH, windowSinkY), 3, nextColor, -1);
			}
		}
	}

void MultipleImagesMatcher::DrawSelections(cv::Mat imageToDraw)
	{
	const cv::Scalar SELECTION_COLOR(80, 150, 210);
	std::vector<BaseTypesWrapper::Point2D>& sourceSelectionList = selectionList.at(currentSourceIndex);
	std::vector<BaseTypesWrapper::Point2D>& sinkSelectionList = selectionList.at(currentSinkIndex);

	for(int pointIndex = 0; pointIndex < sourceSelectionList.size(); pointIndex++)
		{
		BaseTypesWrapper::Point2D& point = sourceSelectionList.at(pointIndex);
		int windowSourceX, windowSourceY;
		bool sourcePointIsVisible = imageZoomingList.at(currentSourceIndex)->ImageToWindowPixel(point.x, point.y, windowSourceX, windowSourceY);
		if (sourcePointIsVisible)
			{
			cv::circle(imageToDraw, cv::Point2d(windowSourceX, windowSourceY), 3, SELECTION_COLOR, -1);
			}
		}

	for(int pointIndex = 0; pointIndex < sinkSelectionList.size(); pointIndex++)
		{
		BaseTypesWrapper::Point2D& point = sinkSelectionList.at(pointIndex);
		int windowSinkX, windowSinkY;
		bool sinkPointIsVisible = imageZoomingList.at(currentSinkIndex)->ImageToWindowPixel(point.x, point.y, windowSinkX, windowSinkY);
		if (sinkPointIsVisible)
			{
			cv::circle(imageToDraw, cv::Point2d(windowSinkX + BASE_WINDOW_WIDTH, windowSinkY), 3, SELECTION_COLOR, -1);
			}
		}
	}

void MultipleImagesMatcher::ExecuteCommand(char command)
	{
	bool deleteIncompleteSelection = false;
	if (command == 'w')
		{
		imageZoomingList.at(currentSourceIndex)->MoveFocusUp();
		}
	else if (command == 's')
		{
		imageZoomingList.at(currentSourceIndex)->MoveFocusDown();
		}
	else if (command == 'a')
		{
		imageZoomingList.at(currentSourceIndex)->MoveFocusLeft();
		}
	else if (command == 'd')
		{
		imageZoomingList.at(currentSourceIndex)->MoveFocusRight();
		}
	else if (command == 'o')
		{
		imageZoomingList.at(currentSourceIndex)->ZoomIn();
		}
	else if (command == 'p')
		{
		imageZoomingList.at(currentSourceIndex)->ZoomOut();
		}
	else if (command == 't')
		{
		imageZoomingList.at(currentSinkIndex)->MoveFocusUp();
		}
	else if (command == 'g')
		{
		imageZoomingList.at(currentSinkIndex)->MoveFocusDown();
		}
	else if (command == 'f')
		{
		imageZoomingList.at(currentSinkIndex)->MoveFocusLeft();
		}
	else if (command == 'h')
		{
		imageZoomingList.at(currentSinkIndex)->MoveFocusRight();
		}
	else if (command == 'k')
		{
		imageZoomingList.at(currentSinkIndex)->ZoomIn();
		}
	else if (command == 'l')
		{
		imageZoomingList.at(currentSinkIndex)->ZoomOut();
		}
	else if (command == 'm')
		{
		SaveCorrespondences();
		}
	else if (command == 'z' && currentSourceIndex > 0)
		{
		currentSourceIndex--;
		deleteIncompleteSelection = true;
		}
	else if (command == 'x' && currentSourceIndex < currentSinkIndex - 1)
		{
		currentSourceIndex++;
		deleteIncompleteSelection = true;
		}
	else if (command == 'b' && currentSourceIndex < currentSinkIndex - 1)
		{
		currentSinkIndex--;
		deleteIncompleteSelection = true;
		}
	else if (command == 'n' && currentSinkIndex < inputImageFilePathList.size() - 1)
		{
		currentSinkIndex++;
		deleteIncompleteSelection = true;
		}

	if (deleteIncompleteSelection)
		{
		PRINT_SOURCE_SINK_INFO;
		std::vector<Correspondence>& correspondencesVector = GetCurrentCorrespondenceVector();
		if (correspondencesVector.size() > 0)
			{
			std::vector<Correspondence>::iterator lastEntry = correspondencesVector.end()-1;
			if (lastEntry->selection != SELECTED_FROM_BOTH)
				{
				correspondencesVector.pop_back();
				}
			}
		}
	}

void MultipleImagesMatcher::SaveCorrespondences()
	{
	CorrespondenceMaps2DSequencePtr correspondenceSequence = NewCorrespondenceMaps2DSequence();

	for(int correspondenceMapIndex = 0; correspondenceMapIndex < correspondenceVectorList.size(); correspondenceMapIndex++)
		{
		std::vector<Correspondence> correspondencesVector = correspondenceVectorList.at(correspondenceMapIndex);
		CorrespondenceMap2DPtr correspondenceMap = NewCorrespondenceMap2D();

		for(int correspondenceIndex = 0; correspondenceIndex < correspondencesVector.size(); correspondenceIndex++)
			{
			Correspondence& correspondence = correspondencesVector.at(correspondenceIndex);
			if (correspondence.selection == SELECTED_FROM_BOTH)
				{
				BaseTypesWrapper::Point2D sourcePoint, sinkPoint;
				sourcePoint.x = correspondence.sourceX;
				sourcePoint.y = correspondence.sourceY;
				sinkPoint.x = correspondence.sinkX;
				sinkPoint.y = correspondence.sinkY;
				AddCorrespondence(*correspondenceMap, sourcePoint, sinkPoint, 1);
				}
			}
		AddCorrespondenceMap(*correspondenceSequence, *correspondenceMap);
		}

	CorrespondenceMaps2DSequenceToMatConverter converter;
	cv::Mat measurementMatrix = converter.Convert(correspondenceSequence);

	cv::FileStorage opencvFile(outputCorrespondencesFilePath, cv::FileStorage::WRITE);
	opencvFile << "MeasurementMatrix" << measurementMatrix;
	opencvFile.release();
	}

void MultipleImagesMatcher::LoadCorrespondences()
	{
	cv::Mat measurementMatrix;

	try 
		{
		cv::FileStorage opencvFile(outputCorrespondencesFilePath, cv::FileStorage::READ);
		opencvFile["MeasurementMatrix"] >> measurementMatrix;
		opencvFile.release();
		}
	catch (...)
		{
		//If reading fails, just overwrite the file.
		return;
		}
	if (measurementMatrix.cols == 0 && measurementMatrix.rows == 0)
		{
		return;
		}

	MatToCorrespondenceMaps2DSequenceConverter converter;
	CorrespondenceMaps2DSequenceConstPtr correspondenceSequence = converter.Convert(measurementMatrix);

	ASSERT( GetNumberOfCorrespondenceMaps(*correspondenceSequence) == correspondenceVectorList.size(), "Error, mismatch between existing file and number of input images");
	int sourceImageIndex = 0;
	int sinkImageIndex = 1;
	int numberOfImages = (1 + std::sqrt(1 + 8 * correspondenceVectorList.size() ) ) / 2;
	for(int correspondenceMapIndex = 0; correspondenceMapIndex < correspondenceVectorList.size(); correspondenceMapIndex++)
		{
		std::vector<Correspondence>& correspondencesVector = correspondenceVectorList.at(correspondenceMapIndex);
		const CorrespondenceMap2D& correspondenceMap = GetCorrespondenceMap(*correspondenceSequence, correspondenceMapIndex);

		for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(correspondenceMap); correspondenceIndex++)
			{
			BaseTypesWrapper::Point2D sourcePoint = GetSource(correspondenceMap, correspondenceIndex);
			BaseTypesWrapper::Point2D sinkPoint = GetSink(correspondenceMap, correspondenceIndex);
			Correspondence correspondence;
			correspondence.sourceX = sourcePoint.x;
			correspondence.sourceY = sourcePoint.y;
			correspondence.sinkX = sinkPoint.x;
			correspondence.sinkY = sinkPoint.y;
			correspondence.selection = SELECTED_FROM_BOTH;
			correspondencesVector.push_back(correspondence);
			AddToSelectionList(sourcePoint.x, sourcePoint.y, selectionList.at(sourceImageIndex));
			AddToSelectionList(sinkPoint.x, sinkPoint.y, selectionList.at(sinkImageIndex));
			}
		if (sinkImageIndex < numberOfImages - 1)
			{
			sinkImageIndex++;
			}
		else
			{
			sourceImageIndex++;
			sinkImageIndex = sourceImageIndex+1;
			}
		}
	}

std::vector<MultipleImagesMatcher::Correspondence>& MultipleImagesMatcher::GetCurrentCorrespondenceVector()
	{
	//ComputeIndex:
	int correspondenceIndex = -1;
	for(int sourceIndex = 0; sourceIndex < currentSourceIndex; sourceIndex++)
		{
		correspondenceIndex += (inputImageFilePathList.size() - 1 - sourceIndex);
		}
	correspondenceIndex += (currentSinkIndex - currentSourceIndex);

	ASSERT( correspondenceIndex >= 0 && correspondenceIndex < correspondenceVectorList.size(), "Error, wrong index computation");
	return correspondenceVectorList.at(correspondenceIndex);
	}

void MultipleImagesMatcher::GetClosePoint(const std::vector<BaseTypesWrapper::Point2D>& pointVector, int& pointX, int& pointY)
	{
	static const int PIXEL_DISTANCE = 5;

	for(int pointIndex = 0; pointIndex < pointVector.size(); pointIndex++)
		{
		int x = pointVector.at(pointIndex).x;
		int y = pointVector.at(pointIndex).y;
	
		if ( x - PIXEL_DISTANCE <= pointX && pointX <= x + PIXEL_DISTANCE && y - PIXEL_DISTANCE <= pointY && pointY <= y + PIXEL_DISTANCE)
			{
			pointX = x;
			pointY = y;
			return;
			}
		}
	}

void MultipleImagesMatcher::AddToSelectionList(int pointX, int pointY, std::vector<BaseTypesWrapper::Point2D>& pointVector)
	{
	bool found = false;
	for(int pointIndex = 0; pointIndex < pointVector.size() && !found; pointIndex++)
		{
		found = (pointVector.at(pointIndex).x == pointX && pointVector.at(pointIndex).y == pointY);
		}

	if (!found)
		{
		BaseTypesWrapper::Point2D newPoint;
		newPoint.x = pointX;
		newPoint.y = pointY;
		pointVector.push_back(newPoint);
		}
	}


}
/** @} */
