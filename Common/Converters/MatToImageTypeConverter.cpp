/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToImageTypeConverter.cpp
 * @date 27/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Types
 * 
 * Implementation of MatToImageTypeConverter class.
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

#include "MatToImageTypeConverter.hpp"
#include <Errors/Assert.hpp>


namespace Types {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ImageType* MatToImageTypeConverter::Convert(const cv::Mat image)
	{
	ASSERT(image.type() ==  CV_8UC3, "MatToImageTypeConverter: image type not supported yet");

	ImageType* asnImage = new ImageType();
	asnImage->height = image.rows;
	asnImage->width = image.cols;
	asnImage->image_mode = ImageMode_mode_rgb;
	asnImage->data.nCount = (int) (image.rows * image.cols * 3);
	//asnImage->data.arr = &(new T_UInt8[asnImage->data.nCount]);

	for(int rowIndex = 0; rowIndex < image.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < image.cols; columnIndex++)
			{
			int dataIndex = rowIndex * image.cols + columnIndex;
			cv::Vec3b pixel = image.at<cv::Vec3b>(rowIndex, columnIndex);
			asnImage->data.arr[3*dataIndex] = (uint8_t) pixel[0];
			asnImage->data.arr[3*dataIndex+1] = (uint8_t) pixel[1];
			asnImage->data.arr[3*dataIndex+2] = (uint8_t) pixel[2];
			}
		}
	
	return asnImage;
	}

}

/** @} */
