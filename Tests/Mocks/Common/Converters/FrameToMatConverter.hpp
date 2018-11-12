/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FrameToMatConverter.hpp
 * @date 08/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * This is a mock for the converter from frame to cv matrix
 * 
 * 
 * @{
 */

#ifndef MOCKS_FRAME_TO_MAT_CONVERTER_HPP
#define MOCKS_FRAME_TO_MAT_CONVERTER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Mocks/Mock.hpp"
#include <Converters/FrameToMatConverter.hpp>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class FrameToMatConverter : public Mock, public Converters::FrameToMatConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual ~FrameToMatConverter();
		const cv::Mat Convert(const FrameWrapper::FrameConstPtr& frame);

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

/* FrameToMatConverter.hpp */
/** @} */
