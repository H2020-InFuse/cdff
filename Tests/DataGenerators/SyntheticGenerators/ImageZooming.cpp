/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageZooming.cpp
 * @date 07/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Implementation of the ImageZooming class.
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
#include "ImageZooming.hpp"
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <Errors/Assert.hpp>

#define MAXIMUM(a, b) ( (a) > (b) ? (a) : (b) )
#define MINIMUM(a, b) ( (a) < (b) ? (a) : (b) ) 

namespace DataGenerators
{

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ImageZooming::ImageZooming(int imageWidth, int imageHeight, int displayWidth, int displayHeight)
	{
	this->imageWidth = imageWidth;
	this->imageHeight = imageHeight;

	this->displayWidth = displayWidth;
	this->displayHeight = displayHeight;

	scale = 1;
	zoomWindowHeight = displayHeight / scale;
	zoomWindowWidth = displayWidth / scale;

	offsetWidth = 0;
	offsetHeight = 0;
	}

ImageZooming::~ImageZooming()
	{

	}

cv::Mat ImageZooming::ExtractZoomedWindow(cv::Mat inputImage)
	{
	ASSERT(inputImage.rows == imageHeight && inputImage.cols == imageWidth, "Zoom Error, input image size does not match the expected size");
	cv::Mat zoomWindow(displayHeight, displayWidth, CV_8UC3);

	for(int row = offsetHeight; row < offsetHeight + zoomWindowHeight; row++)
		{
		for(int column = offsetWidth; column < offsetWidth + zoomWindowWidth; column++)
			{
			cv::Vec3b pixelToDraw;
			int scaledRowIndex	= (row-offsetHeight)*scale;
			int scaledColumnIndex = (column - offsetWidth)*scale;
			if (row < imageHeight && column < imageWidth)
				{
				pixelToDraw = inputImage.at<cv::Vec3b>(row, column);
				}
			else
				{
				pixelToDraw = cv::Vec3b(0, 0, 0);
				}
			DrawZoomPixel(zoomWindow, scaledRowIndex, scaledColumnIndex, pixelToDraw);
			}
		}
	
	return zoomWindow;
	}

cv::Mat ImageZooming::ExtractZoomedWindow(cv::Mat inputImage, cv::Mat overlayImage)
	{
	ASSERT( inputImage.size() == overlayImage.size(), "Zoom Error, input image and overlay image must have the same size");
	ASSERT( inputImage.type() == CV_8UC3 && overlayImage.type() == CV_8UC4, "Zoom Error, input image type has to be CV_8UC3 and overlayImage.type has to be CV_8UC4");

	cv::Mat bufferImage(inputImage.size(), CV_8UC3);
	for(int row = offsetHeight; row < offsetHeight + zoomWindowHeight; row++)
		{
		for(int column = offsetWidth; column < offsetWidth + zoomWindowWidth; column++)
			{
			cv::Vec3b originalPixel = inputImage.at<cv::Vec3b>(row, column);
			cv::Vec4b overlayPixel = overlayImage.at<cv::Vec4b>(row, column);
			float alpha = ((float)overlayPixel[3]) / 255.0;
			for(unsigned colorIndex = 0; colorIndex < 3; colorIndex++)
				{
				bufferImage.at<cv::Vec3b>(row, column)[colorIndex] = (1-alpha)*((float)originalPixel[colorIndex]) + alpha*((float)overlayPixel[colorIndex]);
				}
			}
		}

	return ExtractZoomedWindow(bufferImage);	
	}

cv::Mat ImageZooming::ExtractZoomedWindow(cv::Mat inputImage, cv::Mat overlayImage, float alpha)
	{
	ASSERT( inputImage.size() == overlayImage.size() && inputImage.type() == overlayImage.type(), "Zoom Error, input image and overlay image must have the same size and type");
	ASSERT( inputImage.channels() == overlayImage.channels(), "Zoom Error, overlay image must have the same number of channels compared to the inputImage");
	ASSERT( 0 <= alpha && alpha <=1, "Zoom Error, alpha needs to be a number between 0 and 1");

	cv::Mat bufferImage = (alpha-1)*inputImage + alpha*overlayImage;
	return ExtractZoomedWindow(bufferImage);
	}
	
void ImageZooming::ZoomIn()
	{
	if (scale >= MAXIMUM_SCALE)
		{
		return;
		}

	scale++;
	zoomWindowHeight = displayHeight / scale;
	zoomWindowWidth = displayWidth / scale;
	}

void ImageZooming::ZoomOut()
	{
	if (scale <= 1)
		{
		return;
		}

	scale--;
	zoomWindowHeight = displayHeight / scale;
	zoomWindowWidth = displayWidth / scale;

	if ( offsetHeight + zoomWindowHeight > imageHeight )
		{
		offsetHeight = MAXIMUM(imageHeight - zoomWindowHeight, 0);
		}

	if ( offsetWidth + zoomWindowWidth > imageWidth )
		{
		offsetWidth = MAXIMUM(imageWidth - zoomWindowWidth, 0);
		}
	}

void ImageZooming::MoveFocusLeft()
	{
	offsetWidth = MAXIMUM(offsetWidth - zoomWindowWidth, 0);
	}

void ImageZooming::MoveFocusRight()
	{
	offsetWidth = MINIMUM(offsetWidth + zoomWindowWidth, MAXIMUM(imageWidth - zoomWindowWidth, 0) );
	}

void ImageZooming::MoveFocusUp()
	{
	offsetHeight = MAXIMUM(offsetHeight - zoomWindowHeight, 0);
	}

void ImageZooming::MoveFocusDown()
	{
	offsetHeight = MINIMUM(offsetHeight + zoomWindowHeight, MAXIMUM(imageHeight - zoomWindowHeight, 0) );
	}

bool ImageZooming::WindowToImagePixel(int windowX, int windowY, int& imageX, int& imageY)
	{
	imageX = windowX / scale + offsetWidth;
	imageY = windowY / scale + offsetHeight;

	return (imageX < imageWidth && imageY < imageHeight);
	}

bool ImageZooming::ImageToWindowPixel(int imageX, int imageY, int& windowX, int& windowY)
	{
	if (!PixelIsWithinZoomedWindow(imageX, imageY))
		{
		return false;
		}

	windowX = (imageX - offsetWidth) * scale;
	windowY = (imageY - offsetHeight) * scale;

	return true;
	}

bool ImageZooming::PixelIsWithinZoomedWindow(int imageX, int imageY)
	{
	bool xCoordinateWithinZoomedWindow = (imageX >= offsetWidth && imageX < offsetWidth + zoomWindowWidth && imageX < imageWidth);
	bool yCoordinateWithinZoomedWindow = (imageY >= offsetHeight && imageY < offsetHeight + zoomWindowHeight && imageY < imageHeight);
	return (xCoordinateWithinZoomedWindow && yCoordinateWithinZoomedWindow);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const int ImageZooming::MAXIMUM_SCALE = 4;		


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void ImageZooming::DrawZoomPixel(cv::Mat zoomWindow, int scaledRowIndex, int scaledColumnIndex, const cv::Vec3b& originalPixel)
	{
	for(int scaleIndexX = 0; scaleIndexX < scale; scaleIndexX++)
		{
		for(int scaleIndexY = 0; scaleIndexY < scale; scaleIndexY++)
			{
			zoomWindow.at<cv::Vec3b>( scaledRowIndex + scaleIndexX, scaledColumnIndex + scaleIndexY ) = originalPixel; 
			}
		}
	}


}
/** @} */
