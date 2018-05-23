/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file FrameToMatConverter.hpp
 * @date 08/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from Frame to Mat.
 *  
 *
 * @{
 */

#ifndef FRAME_TO_MAT_CONVERTER_HPP
#define FRAME_TO_MAT_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Frame.hpp>
#include <opencv2/core/core.hpp>

namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class FrameToMatConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual const cv::Mat Convert(const FrameWrapper::FrameConstPtr& frame);
		const cv::Mat ConvertShared(const FrameWrapper::FrameSharedConstPtr& frame);

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
		const cv::Mat ConvertRGB(const FrameWrapper::FrameConstPtr& frame);
		const cv::Mat ConvertGrayscale(const FrameWrapper::FrameConstPtr& frame);
	};

}

#endif

/* FrameToMatConverter.hpp */
/** @} */
