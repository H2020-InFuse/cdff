/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file MatToFrameConverter.hpp
 * @date 08/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from Mat to Frame.
 *  
 *
 * @{
 */

#ifndef MAT_TO_FRAME_CONVERTER_HPP
#define  MAT_TO_FRAME_CONVERTER_HPP


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
class MatToFrameConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual FrameWrapper::FrameConstPtr Convert(const cv::Mat& image);
		virtual void Convert(const cv::Mat& image, FrameWrapper::Frame& conversion);

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
	};

}

#endif

/* MatToFrameConverter.hpp */
/** @} */
