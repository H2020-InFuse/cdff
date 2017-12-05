/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageTypeToMatConverter.cpp
 * @date 20/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Types
 * 
 * Implementation of ImageTypeToMatConverter.
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

#include "ImageTypeToMatConverter.hpp"
#include <Errors/Assert.hpp>


namespace Types {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
cv::Mat ImageTypeToMatConverter::Convert(const ImageType* image)
	{
	ASSERT(image->image_mode == ImageMode_mode_rgb, "ImageTypeToMatConverter: image type not supported yet");
	ASSERT(image->height * image->width * 3 == image->data.nCount, "ImageTypeToMatConverter: input image data is invalid");

	cv::Mat cvImage( image->height, image->width, CV_8UC3, cv::Scalar(0,0,0) );
	for(int rowIndex = 0; rowIndex < cvImage.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < cvImage.cols; columnIndex++)
			{
			int dataIndex = rowIndex * cvImage.cols + columnIndex;
			cvImage.at<cv::Vec3b>( rowIndex, columnIndex )[0] = image->data.arr[3*dataIndex + 0];
			cvImage.at<cv::Vec3b>( rowIndex, columnIndex )[1] = image->data.arr[3*dataIndex + 1];
			cvImage.at<cv::Vec3b>( rowIndex, columnIndex )[2] = image->data.arr[3*dataIndex + 2];
			}
		}	

	return cvImage;
	}

}

/** @} */
