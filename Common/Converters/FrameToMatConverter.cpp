/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FrameToMatConverter.cpp
 * @date 08/12/2017
 * @author Alessandro Bianco
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
	return Convert(*frame);
	}

const cv::Mat FrameToMatConverter::Convert(const FrameWrapper::Frame& frame)
	{
	ASSERT( GetFrameMode(frame) == MODE_RGB, "FrameToMatConverter: Only RGB images are currently supported");
	ASSERT( static_cast<int>( GetFrameHeight(frame) * GetFrameWidth(frame) * 3) == GetNumberOfDataBytes(frame), "FrameToMatConverter: image data size does not match image dimensions.");

	cv::Mat cvImage( GetFrameHeight(frame), GetFrameWidth(frame), CV_8UC3, cv::Scalar(0,0,0) );
	for(int rowIndex = 0; rowIndex < cvImage.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < cvImage.cols; columnIndex++)
			{
			int dataIndex = rowIndex * cvImage.cols + columnIndex;
			cvImage.at<cv::Vec3b>( rowIndex, columnIndex )[0] = GetDataByte(frame, 3*dataIndex + 0);
			cvImage.at<cv::Vec3b>( rowIndex, columnIndex )[1] = GetDataByte(frame, 3*dataIndex + 1);
			cvImage.at<cv::Vec3b>( rowIndex, columnIndex )[2] = GetDataByte(frame, 3*dataIndex + 2);
			}
		}	

	return cvImage;
	}

void FrameToMatConverter::Convert(const FrameWrapper::Frame& frame, cv::Mat& conversion)
	{
	conversion = Convert(frame);
	}

}

/** @} */
