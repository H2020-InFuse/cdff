/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file LinesPainter.cpp
 * @date 09/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Implementation of the LinesPainter class.
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
#include "LinesPainter.hpp"
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
LinesPainter::LinesPainter(std::string inputImageFilePath, std::string outputLinesFilePath)
	{
	this->inputImageFilePath = inputImageFilePath;
	this->outputLinesFilePath = outputLinesFilePath;

	imageZooming = NULL;
	LoadImage();
	LoadLines();

	cv::namedWindow("Lines Painter", 1);
	cv::setMouseCallback("Lines Painter", LinesPainter::MouseCallback, this);
	}

LinesPainter::~LinesPainter()
	{	
	if (imageZooming != NULL)
		{
		delete(imageZooming);
		}
	}

void LinesPainter::Run()
	{
	ASSERT(originalImage.rows > 0 && originalImage.cols >0, "Lines Painter error, the input image is empty");

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
const int LinesPainter::BASE_WINDOW_WIDTH = 800;
const int LinesPainter::BASE_WINDOW_HEIGHT = 600;
const std::vector<cv::Scalar> LinesPainter::COLORS_LIST =
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
void LinesPainter::LoadImage()
	{
	originalImage = cv::imread(inputImageFilePath, CV_LOAD_IMAGE_COLOR);

	if (imageZooming != NULL)
		{
		delete(imageZooming);
		}
	imageZooming = new ImageZooming(originalImage.cols, originalImage.rows, BASE_WINDOW_WIDTH, BASE_WINDOW_HEIGHT);
	}

void LinesPainter::MouseCallback(int event, int x, int y, int flags, void* userdata)
	{
	((LinesPainter*)userdata)->MouseCallback(event, x, y); 
	}

void LinesPainter::MouseCallback(int event, int x, int y)
	{
	if (event != cv::EVENT_LBUTTONDOWN && event != cv::EVENT_RBUTTONDOWN && event != cv::EVENT_MBUTTONDOWN)
		{
		return;
		}

	//If the right mouse button is pressed, delete the last point or line.
	if (event ==  cv::EVENT_RBUTTONDOWN)
		{
		if (linesList.size() > 0)
			{
			if ( (linesList.end()-1)->size() > 0 )
				{
				(linesList.end()-1)->pop_back();
				}
			else
				{
				linesList.pop_back();
				}
			}
		}

	// if the middle mouse button is pressed start a new line.
	if (event == cv::EVENT_MBUTTONDOWN)
		{
		Line newLine;
		linesList.push_back(newLine);
		}

	//Compute the effective position of the selected point coordinates in the image matrix.
	int effectiveX, effectiveY;
	bool validPoint = imageZooming->WindowToImagePixel(x, y, effectiveX, effectiveY);
	if (!validPoint)
		{
		return;
		}
	
	Point newPoint;
	newPoint.x = effectiveX;
	newPoint.y = effectiveY;

	// if the left mouse button is pressed add a new point to the line.
	if (event == cv::EVENT_LBUTTONDOWN)
		{
		if (linesList.size() == 0)
			{
			Line newLine;
			linesList.push_back(newLine);
			}
		(linesList.end()-1)->push_back(newPoint);
		}
	}

void LinesPainter::DrawImage()
	{
	cv::Mat imageToDraw = imageZooming->ExtractZoomedWindow(originalImage);
	DrawLinesOnImage(imageToDraw);
	cv::imshow("Lines Painter", imageToDraw);
	}

void LinesPainter::ExecuteCommand(char command)
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
		SaveLines();
		}
	}

void LinesPainter::SaveLines()
	{
	int numberOfPoints = ComputeNumberOfPoints();
	cv::Mat linesMatrix(numberOfPoints, 3, CV_16UC1);
	
	int totalPointIndex = 0;
	for(int lineIndex = 0; lineIndex < linesList.size(); lineIndex++)
		{
		Line& currentLine = linesList.at(lineIndex);
		for(int pointIndex = 0; pointIndex < currentLine.size(); pointIndex++)
			{
			linesMatrix.at<uint16_t>(totalPointIndex, 0) = currentLine.at(pointIndex).x;
			linesMatrix.at<uint16_t>(totalPointIndex, 1) = currentLine.at(pointIndex).y;
			linesMatrix.at<uint16_t>(totalPointIndex, 2) = lineIndex;
			totalPointIndex++;
			}
		}	

	cv::FileStorage opencvFile(outputLinesFilePath, cv::FileStorage::WRITE);
	opencvFile << "LinesMatrix" << linesMatrix;
	opencvFile.release();
	}

void LinesPainter::LoadLines()
	{
	cv::Mat linesMatrix;

	try 
		{
		cv::FileStorage opencvFile(outputLinesFilePath, cv::FileStorage::READ);
		opencvFile["LinesMatrix"] >> linesMatrix;
		opencvFile.release();
		}
	catch (...)
		{
		//If reading fails, just overwrite the file.
		return;
		}

	if (linesMatrix.rows == 0)
		{
		return;
		}
	ASSERT(linesMatrix.cols == 3 && linesMatrix.type() == CV_16UC1, "Error, output xml file contains some data but the format is incorrect");

	double minLineIndex, maxLineIndex;
	cv::minMaxLoc(linesMatrix(cv::Rect(2, 0, 1, linesMatrix.rows)), &minLineIndex, &maxLineIndex);
	ASSERT(minLineIndex == 0, "Error, output xml file contains some data but the format is incorrect");	

	//Add one more line tha needed
	for(int lineIndex = 0; lineIndex < maxLineIndex + 1; lineIndex++)
		{
		Line newLine;
		linesList.push_back(newLine);
		}
	
	for(int pointIndex = 0; pointIndex < linesMatrix.rows; pointIndex++)
		{
		uint16_t lineIndex = linesMatrix.at<uint16_t>(pointIndex, 2);
		Point newPoint;
		newPoint.x = linesMatrix.at<uint16_t>(pointIndex, 0);
		newPoint.y = linesMatrix.at<uint16_t>(pointIndex, 1);
		linesList.at(lineIndex).push_back(newPoint);
		}
	}

int LinesPainter::ComputeNumberOfPoints()
	{
	int numberOfPoints = 0;
	for(int lineIndex = 0; lineIndex < linesList.size(); lineIndex++)
		{
		numberOfPoints += linesList.at(lineIndex).size();
		}
	return numberOfPoints;
	}

void LinesPainter::DrawLinesOnImage(cv::Mat imageToDraw)
	{
	for(int lineIndex = 0; lineIndex < linesList.size(); lineIndex++)
		{
		Line& currentLine = linesList.at(lineIndex);
		cv::Scalar nextColor = COLORS_LIST.at(lineIndex % COLORS_LIST.size());
		for(int pointIndex = 0; pointIndex < currentLine.size(); pointIndex++)
			{
			Point& point = currentLine.at(pointIndex);
			int windowX, windowY;
			bool pointIsVisible = imageZooming->ImageToWindowPixel(point.x, point.y, windowX, windowY);
			if (pointIsVisible)
				{
				cv::circle(imageToDraw, cv::Point2d(windowX, windowY), 3, nextColor, -1);
				}
			}
		for(int pointIndex = 1; pointIndex < currentLine.size(); pointIndex++)
			{
			Point& point1 = currentLine.at(pointIndex-1);
			Point& point2 = currentLine.at(pointIndex);
			int windowX1, windowY1, windowX2, windowY2;
			bool point1IsVisible = imageZooming->ImageToWindowPixel(point1.x, point1.y, windowX1, windowY1);
			bool point2IsVisible = imageZooming->ImageToWindowPixel(point2.x, point2.y, windowX2, windowY2);
			if (point1IsVisible && point2IsVisible)
				{
				cv::line(imageToDraw, cv::Point2d(windowX1, windowY1), cv::Point2d(windowX2, windowY2), nextColor, 1, 8);
				}
			}
		}
	}
}
/** @} */
