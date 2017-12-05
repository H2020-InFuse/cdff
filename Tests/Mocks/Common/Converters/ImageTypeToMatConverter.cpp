/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageTypeToMatConverter.cpp
 * @date 21/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
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
#include <Mocks/MockMacro.hpp>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ImageTypeToMatConverter::~ImageTypeToMatConverter()
	{

	}

cv::Mat ImageTypeToMatConverter::Convert(const ImageType* image)
	MOCK_METHOD(Types::ImageTypeToMatConverter, Convert, cv::Mat, (image) )


}
/** @} */
