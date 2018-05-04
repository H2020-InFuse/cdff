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
ImagePainter::ImagePainter(std::string inputImageFilePath, std::string outputImageFilePath)
	{
	this->inputImageFilePath = inputImageFilePath;
	this->outputImageFilePath = outputImageFilePath;

	LoadImage();

	cv::namedWindow("Image Painter", 1);
	cv::setMouseCallback("Image Painter", ImagePainter::MouseCallback, this);

	}

ImagePainter::~ImagePainter()
	{

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


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void ImagePainter::LoadImage()
	{
	originalImage = cv::imread(inputImageFilePath, CV_LOAD_IMAGE_COLOR);
	overlayImage = cv::Mat(originalImage.rows, originalImage.cols, CV_8UC3, cv::Scalar(255, 255, 255));

	windowWidth = BASE_WINDOW_WIDTH;
	windowHeight = BASE_WINDOW_HEIGHT;
	scale = 1;
	offsetWidth = 0;
	offsetHeight = 0;
	}

void ImagePainter::MouseCallback(int event, int x, int y, int flags, void* userdata)
	{
	((ImagePainter*)userdata)->MouseCallback(event, x, y); 
	}

void ImagePainter::MouseCallback(int event, int x, int y)
	{
	static bool leftButtonDown = false;
	static bool rightButtonDown = false;
	if (event != cv::EVENT_LBUTTONDOWN && event != cv::EVENT_RBUTTONDOWN && !leftButtonDown && !rightButtonDown)
		{
		return;
		}

	int effectiveColumn = x / scale + offsetWidth;
	int effectiveRow = y / scale + offsetHeight;

	if (event == cv::EVENT_LBUTTONDOWN || (leftButtonDown && event ==  cv::EVENT_MOUSEMOVE) )
		{
		overlayImage.at<cv::Vec3b>(effectiveRow, effectiveColumn)[0] = 0;
		leftButtonDown = true;
		}
	else if (event == cv::EVENT_RBUTTONDOWN || (rightButtonDown && event == cv::EVENT_MOUSEMOVE) )
		{
		overlayImage.at<cv::Vec3b>(effectiveRow, effectiveColumn)[0] = 255;
		rightButtonDown = true;
		}
	else if (event == cv::EVENT_LBUTTONUP)
		{
		leftButtonDown = false;
		}
	else if (event == cv::EVENT_RBUTTONUP)
		{
		rightButtonDown = false;
		}
	}

void ImagePainter::DrawImage()
	{
	imageToDraw = cv::Mat( scale*windowHeight, scale*windowWidth, CV_8UC3);

	for(unsigned row = offsetHeight; row < offsetHeight + windowHeight; row++)
		{
		for(unsigned column = offsetWidth; column < offsetWidth + windowWidth; column++)
			{
			cv::Vec3b originalPixel = originalImage.at<cv::Vec3b>(row, column);
			cv::Vec3b overlayPixel = overlayImage.at<cv::Vec3b>(row, column);
			cv::Vec3b drawPixel;
			if (overlayPixel[0] == 255 && overlayPixel[1] == 255 && overlayPixel[2] == 255)
				{
				drawPixel = originalPixel;
				}
			else
				{
				unsigned sumOfPixelValues = originalPixel[0] + originalPixel[1] + originalPixel[2];
				drawPixel[0] = (sumOfPixelValues > 510 ? 0 : 255);
				drawPixel[1] = drawPixel[0];
				drawPixel[2] = drawPixel[0];
				}
			
			for(unsigned scaleIndexX = 0; scaleIndexX < scale; scaleIndexX++)
				{
				for(unsigned scaleIndexY = 0; scaleIndexY < scale; scaleIndexY++)
					{
					imageToDraw.at<cv::Vec3b>( (row-offsetHeight)*scale + scaleIndexX, (column - offsetWidth)*scale + scaleIndexY ) = drawPixel; 
					}
				}
			}
		}

	cv::imshow("Image Painter", imageToDraw);
	}

void ImagePainter::ExecuteCommand(char command)
	{
	if (command == 'w')
		{
		int candidateOffset = offsetHeight - windowHeight;
		offsetHeight = (candidateOffset >= 0 ? candidateOffset : 0);
		}
	else if (command == 's')
		{
		int candidateOffset = offsetHeight + windowHeight;
		offsetHeight = (candidateOffset + windowHeight <= originalImage.rows ? candidateOffset : (originalImage.rows - windowHeight) );
		}
	else if (command == 'a')
		{
		int candidateOffset = offsetWidth - windowWidth;
		offsetWidth = (candidateOffset >= 0 ? candidateOffset : 0);
		}
	else if (command == 'd')
		{
		int candidateOffset = offsetWidth + windowWidth;
		offsetWidth = (candidateOffset + windowWidth <= originalImage.cols ? candidateOffset : (originalImage.cols - windowWidth) );
		}
	else if (command == 'o' && scale < 4)
		{
		scale++;
		windowHeight = BASE_WINDOW_HEIGHT / scale;
		windowWidth = BASE_WINDOW_WIDTH / scale;
		}
	else if (command == 'p' && scale > 1)
		{
		scale--;
		windowHeight = BASE_WINDOW_HEIGHT / scale;
		windowWidth = BASE_WINDOW_WIDTH / scale;
		offsetHeight = (offsetHeight + windowHeight <= originalImage.cols ? offsetHeight : (originalImage.rows - windowHeight) );
		offsetWidth = (offsetWidth + windowWidth <= originalImage.rows ? offsetWidth : (originalImage.cols - windowWidth) );
		}
	else if (command == 'm')
		{
		SaveImage();
		}
	}

void ImagePainter::SaveImage()
	{
	cv::Mat imageToSave = originalImage.clone();

	for(unsigned row = 0; row < imageToSave.rows; row++)
		{
		for (unsigned column = 0; column < imageToSave.cols; column++)
			{
			cv::Vec3b originalPixel = imageToSave.at<cv::Vec3b>(row, column);
			cv::Vec3b overlayPixel = overlayImage.at<cv::Vec3b>(row, column);
			if (overlayPixel[0] != 255 || overlayPixel[1] != 255 || overlayPixel[2] != 255)
				{
				cv::Vec3b substitutePixel(255, 255, 255);
				imageToSave.at<cv::Vec3b>(row, column) = substitutePixel;
				}
			else if (originalPixel[0] == 255 && originalPixel[1] == 255 && originalPixel[2] == 255)
				{
				cv::Vec3b substitutePixel(254, 254, 254);
				imageToSave.at<cv::Vec3b>(row, column) = substitutePixel;
				}
			}
		}

	cv::imwrite(outputImageFilePath, imageToSave);
	}


}
/** @} */
