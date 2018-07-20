/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImagePainter.cpp
 * @date 04/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Implementation of the ImagePainter class.
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
#include "ImagePainter.hpp"
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <Errors/Assert.hpp>

namespace DataGenerators
{

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ImagePainter::ImagePainter(std::string inputImageFilePath, std::string outputKeypointsFilePath)
	{
	this->inputImageFilePath = inputImageFilePath;
	this->outputKeypointsFilePath = outputKeypointsFilePath;

	imageZooming = NULL;
	LoadImage();
	LoadKeypointsImage();
	boxSelection = false;

	cv::namedWindow("Image Painter", 1);
	cv::setMouseCallback("Image Painter", ImagePainter::MouseCallback, this);
	}

ImagePainter::~ImagePainter()
	{	
	if (imageZooming != NULL)
		{
		delete(imageZooming);
		}
	}

void ImagePainter::Run()
	{
	ASSERT(originalImage.rows > 0 && originalImage.cols >0, "Image Painter error, the input image is empty");

	bool painterIsActive = true;
	while(painterIsActive)
		{
		DrawImage();
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
const int ImagePainter::BASE_WINDOW_WIDTH = 1200;
const int ImagePainter::BASE_WINDOW_HEIGHT = 900;
const std::vector<cv::Scalar> ImagePainter::COLORS_LIST =
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
void ImagePainter::LoadImage()
	{
	originalImage = cv::imread(inputImageFilePath, CV_LOAD_IMAGE_COLOR);

	if (imageZooming != NULL)
		{
		delete(imageZooming);
		}
	imageZooming = new ImageZooming(originalImage.cols, originalImage.rows, BASE_WINDOW_WIDTH, BASE_WINDOW_HEIGHT);
	}

void ImagePainter::MouseCallback(int event, int x, int y, int flags, void* userdata)
	{
	((ImagePainter*)userdata)->MouseCallback(event, x, y); 
	}

void ImagePainter::MouseCallback(int event, int x, int y)
	{
	if (event != cv::EVENT_LBUTTONDOWN && event != cv::EVENT_RBUTTONDOWN)
		{
		return;
		}

	//If right mouse button is pressed remove last added keypoint
	if (event == cv::EVENT_RBUTTONDOWN)
		{
		if (keypointsList.size() > 0)
			{
			keypointsList.pop_back();
			}
		return;
		}

	//Compute the effective position of the selected point coordinates in the image matrix.
	int effectiveX, effectiveY;
	bool validPoint = imageZooming->WindowToImagePixel(x, y, effectiveX, effectiveY);
	if (!validPoint)
		{
		return;
		}

	//Add a new point
	if (!boxSelection)
		{
		Point newPoint;
		newPoint.x = effectiveX;
		newPoint.y = effectiveY;
		keypointsList.push_back(newPoint);
		return;
		}

	//Start adding square
	if (!topLeftCornerSelected)
		{
		topLeftCornerSelected = true;
		topLeftCorner.x = effectiveX;
		topLeftCorner.y = effectiveY;
		return;
		}

	//Add the square
	const int CLOSENESS = 3;
	int beginX = (topLeftCorner.x < effectiveX ? topLeftCorner.x : effectiveX);
	int endX = (topLeftCorner.x >= effectiveX ? topLeftCorner.x : effectiveX);
	int beginY = (topLeftCorner.y < effectiveY ? topLeftCorner.y : effectiveY);
	int endY = (topLeftCorner.y >= effectiveY ? topLeftCorner.y : effectiveY);
	int currentKeypointNumber = keypointsList.size();
	for(int selectionX = beginX; selectionX <= endX; selectionX += CLOSENESS)
		{
		for(int selectionY = beginY; selectionY <= endY; selectionY += CLOSENESS)
			{
			Point newPoint;
			newPoint.x = selectionX;
			newPoint.y = selectionY;
			bool pointFound = false;
			for(int pointIndex = 0; pointIndex < currentKeypointNumber && !pointFound; pointIndex++)
				{
				const Point& oldPoint = keypointsList.at(pointIndex);
				pointFound = (oldPoint.x - CLOSENESS <= newPoint.x && newPoint.x <= oldPoint.x + CLOSENESS);
				pointFound = pointFound && (oldPoint.y - CLOSENESS <= newPoint.y && newPoint.y <= oldPoint.y + CLOSENESS);
				}
			if (!pointFound)
				{
				keypointsList.push_back(newPoint);
				}
			}
		}
	topLeftCornerSelected = false;
	}

void ImagePainter::DrawImage()
	{
	cv::Mat imageToDraw = imageZooming->ExtractZoomedWindow(originalImage);
	DrawKeypoints(imageToDraw);
	cv::imshow("Image Painter", imageToDraw);
	}

void ImagePainter::DrawKeypoints(cv::Mat imageToDraw)
	{
	for(int pointIndex = 0; pointIndex < keypointsList.size(); pointIndex++)
		{
		Point& point = keypointsList.at(pointIndex);
		int windowX, windowY;
		bool pointIsVisible = imageZooming->ImageToWindowPixel(point.x, point.y, windowX, windowY);
		if (pointIsVisible)
			{
			const cv::Scalar& nextColor = COLORS_LIST.at(pointIndex % COLORS_LIST.size());
			cv::circle(imageToDraw, cv::Point2d(windowX, windowY), 3, nextColor, -1);
			}			
		}
	}

void ImagePainter::ExecuteCommand(char command)
	{
	if (command == 'w')
		{
		imageZooming->MoveFocusUp();
		}
	else if (command == 's')
		{
		imageZooming->MoveFocusDown();
		}
	else if (command == 'a')
		{
		imageZooming->MoveFocusLeft();
		}
	else if (command == 'd')
		{
		imageZooming->MoveFocusRight();
		}
	else if (command == 'o')
		{
		imageZooming->ZoomIn();
		}
	else if (command == 'p')
		{
		imageZooming->ZoomOut();
		}
	else if (command == 'm')
		{
		SaveKeypointsImage();
		}
	else if (command == 't')
		{
		boxSelection = !boxSelection;
		topLeftCornerSelected = false;
		}
	}

void ImagePainter::SaveKeypointsImage()
	{
	cv::Mat keypointsMatrix(keypointsList.size(), 2, CV_16UC1);
	for(int pointIndex = 0; pointIndex < keypointsList.size(); pointIndex++)
		{
		keypointsMatrix.at<uint16_t>(pointIndex, 0) = keypointsList.at(pointIndex).x;
		keypointsMatrix.at<uint16_t>(pointIndex, 1) = keypointsList.at(pointIndex).y;
		}

	cv::FileStorage opencvFile(outputKeypointsFilePath, cv::FileStorage::WRITE);
	opencvFile << "KeypointsMatrix" << keypointsMatrix;
	opencvFile.release();
	}

void ImagePainter::LoadKeypointsImage()
	{
	cv::Mat keypointsMatrix;
	try 
		{
		cv::FileStorage opencvFile(outputKeypointsFilePath, cv::FileStorage::READ);
		opencvFile["KeypointsMatrix"] >> keypointsMatrix;
		opencvFile.release();
		}
	catch (...)
		{
		//If reading fails, just overwrite the file.
		return;
		}

	if (keypointsMatrix.rows == 0)
		{
		return;
		}
	ASSERT(keypointsMatrix.cols == 2 && keypointsMatrix.type() == CV_16UC1, "Error, output xml file contains some data but the format is incorrect");

	for(int pointIndex = 0; pointIndex < keypointsMatrix.rows; pointIndex++)
		{
		Point newPoint;
		newPoint.x = keypointsMatrix.at<uint16_t>(pointIndex, 0);
		newPoint.y = keypointsMatrix.at<uint16_t>(pointIndex, 1);
		keypointsList.push_back(newPoint);
		}	
	}


}
/** @} */
