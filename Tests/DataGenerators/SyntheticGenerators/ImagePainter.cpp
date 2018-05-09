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

	imageZooming = NULL;
	LoadImage();

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


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void ImagePainter::LoadImage()
	{
	originalImage = cv::imread(inputImageFilePath, CV_LOAD_IMAGE_COLOR);
	overlayImage = cv::Mat(originalImage.rows, originalImage.cols, CV_8UC4, cv::Scalar(255, 255, 255, 0));

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
	static bool leftButtonDown = false;
	static bool rightButtonDown = false;
	if (event != cv::EVENT_LBUTTONDOWN && event != cv::EVENT_RBUTTONDOWN && !leftButtonDown && !rightButtonDown)
		{
		return;
		}

	//Compute the effective position of the selected point coordinates in the image matrix.
	int effectiveColumn, effectiveRow;
	imageZooming->WindowToImagePixel(x, y, effectiveColumn, effectiveRow);

	//When the left mouse button is pressed, a new keypoint is selected and added to the overlay with an appropriate color that allows it to be more visible.
	if (event == cv::EVENT_LBUTTONDOWN || (leftButtonDown && event ==  cv::EVENT_MOUSEMOVE) )
		{
		cv::Vec3b originalPixel = originalImage.at<cv::Vec3b>(effectiveRow, effectiveColumn);
		uint8_t grayColorValue = ( originalPixel[0] + originalPixel[1] + originalPixel[2] >= 510 ) ? 0 : 255;

		overlayImage.at<cv::Vec4b>(effectiveRow, effectiveColumn)[0] = grayColorValue;
		overlayImage.at<cv::Vec4b>(effectiveRow, effectiveColumn)[1] = grayColorValue;
		overlayImage.at<cv::Vec4b>(effectiveRow, effectiveColumn)[2] = grayColorValue;
		overlayImage.at<cv::Vec4b>(effectiveRow, effectiveColumn)[3] = 255;
		leftButtonDown = true;
		}
	//When the right mouse button is pressed, a keypoint selection is cancelled and it is removed from the overlay
	else if (event == cv::EVENT_RBUTTONDOWN || (rightButtonDown && event == cv::EVENT_MOUSEMOVE) )
		{
		overlayImage.at<cv::Vec4b>(effectiveRow, effectiveColumn)[0] = 0;
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
	cv::Mat imageToDraw = imageZooming->ExtractZoomedWindow(originalImage, overlayImage);
	cv::imshow("Image Painter", imageToDraw);
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
		SaveImage();
		}
	}

void ImagePainter::SaveImage()
	{
	cv::Mat imageToSave = originalImage.clone();

	for(int row = 0; row < imageToSave.rows; row++)
		{
		for (int column = 0; column < imageToSave.cols; column++)
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
