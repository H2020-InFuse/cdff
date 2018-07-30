/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToFrameConverter.cpp
 * @date 27/11/2017
 * @authors Alessandro Bianco, Xavier Martinez, Nassir W. Oumer
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of MatToFrameConverter class.
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

#include "MatToFrameConverter.hpp"
#include <Errors/Assert.hpp>

namespace Converters {

using namespace FrameWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

FrameConstPtr MatToFrameConverter::Convert(const cv::Mat& image)
	{
	FramePtr frame = new Frame();
	if (image.rows == 0 && image.cols == 0)
		return frame;

	SetFrameSize(*frame, image.cols, image.rows);

	if(image.type()==CV_8UC3)
		{
		ConvertRGB(image, *frame);
		}
	else if(image.type()==CV_8UC1)
		{
		ConvertGrayscale(image, *frame);
		}
	else if(image.type()==CV_32FC1)
		{
		ConvertFloat(image, *frame);
		}
	else
		{
		ASSERT(false, "Unhandled image type in MatToFrameConverter, only CV_8UC1 and CV_8UC3 supported.");
		}
	return frame;
	}


FrameSharedConstPtr MatToFrameConverter::ConvertShared(const cv::Mat& image)
	{
	FrameConstPtr frame = Convert(image);
	FrameSharedConstPtr sharedFrame(frame);
	return sharedFrame;
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void MatToFrameConverter::ConvertRGB(const cv::Mat& image, Frame& frame)
	{
	SetFrameMode(frame, MODE_RGB);		
	
	for(int rowIndex = 0; rowIndex < image.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < image.cols; columnIndex++)
			{
			cv::Vec3b pixel = image.at<cv::Vec3b>(rowIndex, columnIndex);
				AppendData<uint8_t>(frame,  pixel[0]);
				AppendData<uint8_t>(frame,  pixel[1]);
				AppendData<uint8_t>(frame,  pixel[2]);
			}
		}
	}

void MatToFrameConverter::ConvertGrayscale(const cv::Mat& image, Frame& frame)
	{
	SetFrameMode(frame, MODE_GRAYSCALE);
	for(int rowIndex = 0; rowIndex < image.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < image.cols; columnIndex++)
			{
				uchar pixel = image.at<uchar>(rowIndex, columnIndex);
				AppendData<uint8_t>(frame, pixel);
			}
		}
	}

	void MatToFrameConverter::ConvertFloat(const cv::Mat& image, Frame& frame)
	{
		SetFrameMode(frame, MODE_GRAYSCALE);
		for(int rowIndex = 0; rowIndex < image.rows; rowIndex++)
		{
			for(int columnIndex = 0; columnIndex < image.cols; columnIndex++)
			{
				float pixel = image.at<float>(rowIndex, columnIndex);
				AppendData<float>(frame, pixel);
			}
		}
	}

}

/** @} */
