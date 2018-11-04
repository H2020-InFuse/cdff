/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToFrameConverter.hpp
 * @date 19/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * This is a mock for the converter from cv matrix to frame
 * 
 * 
 * @{
 */

#ifndef MOCKS_MAT_TO_FRAME_CONVERTER_HPP
#define MOCKS_MAT_TO_FRAME_CONVERTER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Mocks/Mock.hpp"
#include <Converters/MatToFrameConverter.hpp>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class MatToFrameConverter : public Mock, public Converters::MatToFrameConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual ~MatToFrameConverter();
		const FrameWrapper::FrameConstPtr Convert(const cv::Mat frame);

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
