/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file FrameToImageConverter.hpp
 * @date 02/08/2018
 * @author Vincent Bissonnette
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from Image to Image.
 *  
 *
 * @{
 */

#ifndef FRAME_TO_IMAGE_CONVERTER_HPP
#define FRAME_TO_IMAGE_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Frame.hpp>
#include <Image.hpp>

namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class FrameToImageConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
        virtual const ImageWrapper::ImageConstPtr Convert(const FrameWrapper::FrameConstPtr& frame);
        const ImageWrapper::ImageSharedConstPtr ConvertShared(const FrameWrapper::FrameSharedConstPtr& frame);

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */	
	private:
    const ImageWrapper::ImageMode ConvertFrameModeToImageMode(const FrameWrapper::FrameMode& frameMode);
	};

}

#endif

/* FrameToImageConverter.hpp */
/** @} */
