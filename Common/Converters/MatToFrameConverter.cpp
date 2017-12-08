/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToFrameConverter.cpp
 * @date 08/12/2017
 * @author Alessandro Bianco
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

using namespace CppTypes;

namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
CppTypes::Frame::ConstPtr MatToFrameConverter::Convert(const cv::Mat image)
	{
	ASSERT(image.type() ==  CV_8UC3, "MatToFrameConverter: image type not supported yet");

	CppTypes::Frame::Ptr frame = CppTypes::Frame::Ptr( new Frame() );
	frame->SetFrameSize(image.cols, image.rows);
	frame->SetFrameMode(Frame::MODE_RGB);

	for(int rowIndex = 0; rowIndex < image.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < image.cols; columnIndex++)
			{
			cv::Vec3b pixel = image.at<cv::Vec3b>(rowIndex, columnIndex);
			frame->AddDataByte( (uint8_t) pixel[0] );
			frame->AddDataByte( (uint8_t) pixel[1] );
			frame->AddDataByte( (uint8_t) pixel[2] );
			}
		}
	
	return frame;
	}

}

/** @} */
