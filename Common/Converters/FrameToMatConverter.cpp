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

using namespace CppTypes;

namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
const cv::Mat FrameToMatConverter::Convert(CppTypes::Frame::ConstPtr frame)
	{
	ASSERT(frame->GetFrameMode() == Frame::MODE_RGB, "FrameToMatConverter: image type not supported yet");
	ASSERT( (int) (frame->GetFrameHeight() * frame->GetFrameWidth() * 3) == frame->GetNumberOfDataBytes(), "FrameToMatConverter: input image data is invalid");

	cv::Mat cvImage( frame->GetFrameHeight(), frame->GetFrameWidth(), CV_8UC3, cv::Scalar(0,0,0) );
	for(int rowIndex = 0; rowIndex < cvImage.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < cvImage.cols; columnIndex++)
			{
			int dataIndex = rowIndex * cvImage.cols + columnIndex;
			cvImage.at<cv::Vec3b>( rowIndex, columnIndex )[0] = frame->GetDataByte(3*dataIndex + 0);
			cvImage.at<cv::Vec3b>( rowIndex, columnIndex )[1] = frame->GetDataByte(3*dataIndex + 1);
			cvImage.at<cv::Vec3b>( rowIndex, columnIndex )[2] = frame->GetDataByte(3*dataIndex + 2);
			}
		}	

	return cvImage;
	}

}

/** @} */
