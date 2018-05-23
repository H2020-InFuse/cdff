/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FrameToMatConverter.cpp
 * @date 14/04/2018
 * @author Alessandro Bianco  and  Nassir W. Oumer 
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of FrameToMatConverter.
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

#include "FrameToMatConverter.hpp"
#include <Errors/Assert.hpp>
#include<iostream>

namespace Converters {

using namespace FrameWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

const cv::Mat FrameToMatConverter::Convert(const FrameWrapper::FrameConstPtr& frame)
	{

	if (GetFrameHeight(*frame) == 0 && GetFrameWidth(*frame) == 0)
		{
		return cv::Mat();
		}

  	if(GetFrameMode(*frame)==MODE_RGB)
		{
		return ConvertRGB(frame);
		}
	else if(GetFrameMode(*frame)==MODE_GRAYSCALE)
		{
		return ConvertGrayscale(frame);
		}
	else
		{
		ASSERT(false, "Unhandled frame type in FrameToMatConverter");
		}
	return cv::Mat();
	}


const cv::Mat FrameToMatConverter::ConvertShared(const FrameWrapper::FrameSharedConstPtr& frame)
	{
	return Convert(frame.get());
	} 

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
const cv::Mat FrameToMatConverter::ConvertRGB(const FrameWrapper::FrameConstPtr& frame)
	{

	ASSERT(GetFrameMode(*frame)==MODE_RGB, "ConvertRGB called on a non-rgb frame");
	ASSERT( static_cast<int>( GetFrameHeight(*frame) * GetFrameWidth(*frame) * 3) == GetNumberOfDataBytes(*frame), "FrameToMatConverter: image data size does not match image dimensions.");
	cv::Mat cvImage( GetFrameHeight(*frame), GetFrameWidth(*frame), CV_8UC3, cv::Scalar(0,0,0) );
	ASSERT(!cvImage.empty(), "FrameToMatConverter:  RGB  images are currently supported");
	for(int rowIndex = 0; rowIndex < cvImage.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < cvImage.cols; columnIndex++)
			{
			int dataIndex = rowIndex * cvImage.cols + columnIndex;
			cvImage.at<cv::Vec3b>( rowIndex, columnIndex )[0] = GetDataByte(*frame, 3*dataIndex + 0);
			cvImage.at<cv::Vec3b>( rowIndex, columnIndex )[1] = GetDataByte(*frame, 3*dataIndex + 1);
			cvImage.at<cv::Vec3b>( rowIndex, columnIndex )[2] = GetDataByte(*frame, 3*dataIndex + 2);
			}
		}	

	return cvImage;
	}

const cv::Mat FrameToMatConverter::ConvertGrayscale(const FrameWrapper::FrameConstPtr& frame)
	{

	ASSERT(GetFrameMode(*frame)==MODE_GRAYSCALE, "ConvertGrayscale called on a non-grayscale frame");
       	ASSERT( static_cast<int>( GetFrameHeight(*frame) * GetFrameWidth(*frame)) == GetNumberOfDataBytes(*frame), "FrameToMatConverter: image 	data size does not match image dimensions.");

	cv::Mat cvImage( GetFrameHeight(*frame), GetFrameWidth(*frame), CV_8UC1, cv::Scalar(0) );
	ASSERT(!cvImage.empty(), "FrameToMatConverter:  Empty image, Gray scale  images are currently supported");
	for(int rowIndex = 0; rowIndex < cvImage.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < cvImage.cols; columnIndex++)
			{
			int dataIndex = rowIndex * cvImage.cols + columnIndex;
			cvImage.at<uchar>( rowIndex, columnIndex ) = GetDataByte(*frame, dataIndex);
			}
		}	

	return cvImage;
	}

}

/** @} */
