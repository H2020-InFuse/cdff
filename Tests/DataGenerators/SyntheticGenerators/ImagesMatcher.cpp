/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImagesMatcher.cpp
 * @date 07/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Implementation of the ImagesMatcher class.
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
#include "ImagesMatcher.hpp"
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <Errors/Assert.hpp>

#define DELETE_IF_NOT_NULL(pointer) \
	if (pointer != NULL) \
		{ \
		delete(pointer); \
		}
	

namespace DataGenerators
{

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ImagesMatcher::ImagesMatcher(std::string inputSourceImageFilePath, std::string inputSinkImageFilePath, std::string outputCorrespondencesFilePath)
	{
	this->inputSourceImageFilePath = inputSourceImageFilePath;
	this->inputSinkImageFilePath = inputSinkImageFilePath;
	this->outputCorrespondencesFilePath = outputCorrespondencesFilePath;

	sourceImageZooming = NULL;
	sinkImageZooming = NULL;
	LoadImages();

	cv::namedWindow("Images Matcher", 1);
	cv::setMouseCallback("Images Matcher", ImagesMatcher::MouseCallback, this);
	}

ImagesMatcher::~ImagesMatcher()
	{	
	DELETE_IF_NOT_NULL(sourceImageZooming);
	DELETE_IF_NOT_NULL(sinkImageZooming);
	}

void ImagesMatcher::Run()
	{
	ASSERT(originalSourceImage.rows > 0 && originalSourceImage.cols >0, "Images Matcher error, the input source image is empty");
	ASSERT(originalSinkImage.rows > 0 && originalSinkImage.cols >0, "Images Matcher error, the input sink image is empty");

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
const int ImagesMatcher::BASE_WINDOW_WIDTH = 800;
const int ImagesMatcher::BASE_WINDOW_HEIGHT = 600;
const std::vector<cv::Scalar> ImagesMatcher::COLORS_LIST =
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
void ImagesMatcher::LoadImages()
	{
	originalSourceImage = cv::imread(inputSourceImageFilePath, CV_LOAD_IMAGE_COLOR);
	originalSinkImage = cv::imread(inputSinkImageFilePath, CV_LOAD_IMAGE_COLOR);

	DELETE_IF_NOT_NULL(sourceImageZooming);
	DELETE_IF_NOT_NULL(sinkImageZooming);
	sourceImageZooming = new ImageZooming(originalSourceImage.cols, originalSourceImage.rows, BASE_WINDOW_WIDTH, BASE_WINDOW_HEIGHT);
	sinkImageZooming = new ImageZooming(originalSinkImage.cols, originalSinkImage.rows, BASE_WINDOW_WIDTH, BASE_WINDOW_HEIGHT);
	}

void ImagesMatcher::MouseCallback(int event, int x, int y, int flags, void* userdata)
	{
	((ImagesMatcher*)userdata)->MouseCallback(event, x, y); 
	}

void ImagesMatcher::MouseCallback(int event, int x, int y)
	{
	if (event != cv::EVENT_LBUTTONDOWN && event != cv::EVENT_RBUTTONDOWN)
		{
		return;
		}

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
	if (currentSelection == SELECTED_FROM_SOURCE)
		{
		sourceImageZooming->WindowToImagePixel(x, y, effectiveX, effectiveY);
		ASSERT(effectiveX < originalSourceImage.cols && effectiveY < originalSourceImage.rows, "Computation error in effective coordinates: coordinates exceed source limit");
		}
	else
		{
		sinkImageZooming->WindowToImagePixel(x - BASE_WINDOW_WIDTH, y, effectiveX, effectiveY);
		ASSERT(effectiveX < originalSinkImage.cols && effectiveY < originalSinkImage.rows, "Computation error in effective coordinates: coordinates exceed sink limit");
		}
	ASSERT(effectiveX >= 0 && effectiveY >= 0, "Computation error in effective coordinates: coordinates are negative");

	
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
	if (lastEntry->selection == SELECTED_FROM_SOURCE && currentSelection == SELECTED_FROM_SINK)
		{
		lastEntry->sinkX = effectiveX;
		lastEntry->sinkY = effectiveY;
		lastEntry->selection = SELECTED_FROM_BOTH;
		}
	else if (lastEntry->selection == SELECTED_FROM_SINK && currentSelection == SELECTED_FROM_SOURCE)
		{
		lastEntry->sourceX = effectiveX;
		lastEntry->sourceY = effectiveY;
		lastEntry->selection = SELECTED_FROM_BOTH;
		}
	}

void ImagesMatcher::DrawImages()
	{
	cv::Mat sourceImageToDraw = sourceImageZooming->ExtractZoomedWindow(originalSourceImage);
	cv::Mat sinkImageToDraw = sinkImageZooming->ExtractZoomedWindow(originalSinkImage);
	cv::Mat imageToDraw;
	cv::hconcat(sourceImageToDraw, sinkImageToDraw, imageToDraw);

	DrawCorrespondences(imageToDraw);
	cv::imshow("Images Matcher", imageToDraw);
	}

void ImagesMatcher::DrawCorrespondences(cv::Mat imageToDraw)
	{
	unsigned nextColorIndex = 0;
	for(unsigned index = 0; index < correspondencesVector.size() && correspondencesVector.at(index).selection == SELECTED_FROM_BOTH; index++)
		{
		Correspondence& correspondence = correspondencesVector.at(index);
		cv::Scalar nextColor = COLORS_LIST.at(nextColorIndex % COLORS_LIST.size());

		int windowSourceX, windowSourceY, windowSinkX, windowSinkY;
		bool sourcePointIsVisible = sourceImageZooming->ImageToWindowPixel(correspondence.sourceX, correspondence.sourceY, windowSourceX, windowSourceY);
		bool sinkPointIsVisible = sinkImageZooming->ImageToWindowPixel(correspondence.sinkX, correspondence.sinkY, windowSinkX, windowSinkY);

		if (sourcePointIsVisible && sinkPointIsVisible)
			{
			cv::line(imageToDraw, cv::Point2d(windowSourceX, windowSourceY), cv::Point2d(windowSinkX + BASE_WINDOW_WIDTH, windowSinkY), nextColor, 1, 8);
			nextColorIndex++;
			}
		}

	DrawLastSelectedPoint(imageToDraw, nextColorIndex);
	}

void ImagesMatcher::DrawLastSelectedPoint(cv::Mat imageToDraw, int nextColorIndex)
	{
	if (correspondencesVector.size() == 0)
		{
		return;
		}

	std::vector<Correspondence>::iterator lastEntry = correspondencesVector.end()-1;
	cv::Scalar nextColor = COLORS_LIST.at( nextColorIndex % COLORS_LIST.size());
	if ( lastEntry->selection == SELECTED_FROM_SOURCE )
		{
		int windowSourceX, windowSourceY;
		bool sourcePointIsVisible = sourceImageZooming->ImageToWindowPixel(lastEntry->sourceX, lastEntry->sourceY, windowSourceX, windowSourceY);
		if (sourcePointIsVisible)
			{
			cv::circle(imageToDraw, cv::Point2d(windowSourceX, windowSourceY), 3, nextColor, -1);
			}
		}
	else if ( lastEntry->selection == SELECTED_FROM_SINK )
		{
		int windowSinkX, windowSinkY;
		bool sinkPointIsVisible = sinkImageZooming->ImageToWindowPixel(lastEntry->sinkX, lastEntry->sinkY, windowSinkX, windowSinkY);
		if (sinkPointIsVisible)
			{
			cv::circle(imageToDraw, cv::Point2d(windowSinkX + BASE_WINDOW_WIDTH, windowSinkY), 3, nextColor, -1);
			}
		}
	}

void ImagesMatcher::ExecuteCommand(char command)
	{
	if (command == 'w')
		{
		sourceImageZooming->MoveFocusUp();
		}
	else if (command == 's')
		{
		sourceImageZooming->MoveFocusDown();
		}
	else if (command == 'a')
		{
		sourceImageZooming->MoveFocusLeft();
		}
	else if (command == 'd')
		{
		sourceImageZooming->MoveFocusRight();
		}
	else if (command == 'o')
		{
		sourceImageZooming->ZoomIn();
		}
	else if (command == 'p')
		{
		sourceImageZooming->ZoomOut();
		}
	else if (command == 't')
		{
		sinkImageZooming->MoveFocusUp();
		}
	else if (command == 'g')
		{
		sinkImageZooming->MoveFocusDown();
		}
	else if (command == 'f')
		{
		sinkImageZooming->MoveFocusLeft();
		}
	else if (command == 'h')
		{
		sinkImageZooming->MoveFocusRight();
		}
	else if (command == 'k')
		{
		sinkImageZooming->ZoomIn();
		}
	else if (command == 'l')
		{
		sinkImageZooming->ZoomOut();
		}
	else if (command == 'm')
		{
		SaveCorrespondences();
		}
	}

void ImagesMatcher::SaveCorrespondences()
	{
	cv::Mat correspondenceMatrix( correspondencesVector.size(), 4, CV_16UC1);
	for(int row = 0; row < correspondenceMatrix.rows; row++)
		{
		correspondenceMatrix.at<uint16_t>(row, 0) = correspondencesVector.at(row).sourceX;
		correspondenceMatrix.at<uint16_t>(row, 1) = correspondencesVector.at(row).sourceY;
		correspondenceMatrix.at<uint16_t>(row, 2) = correspondencesVector.at(row).sinkX;
		correspondenceMatrix.at<uint16_t>(row, 3) = correspondencesVector.at(row).sinkY;
		}

	cv::FileStorage opencvFile(outputCorrespondencesFilePath, cv::FileStorage::WRITE);
	opencvFile << "CorrespondenceMap" << correspondenceMatrix;
	opencvFile.release();
	}


}
/** @} */
