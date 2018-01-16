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
 * @addtogroup Mocks
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
#include <Mocks/MockMacro.hpp>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
FrameToMatConverter::~FrameToMatConverter()
	{

	}

const cv::Mat FrameToMatConverter::Convert(const FrameWrapper::FrameConstPtr& frame)
	MOCK_METHOD(Converters::FrameToMatConverter, Convert, cv::Mat, (frame) )

const cv::Mat FrameToMatConverter::Convert(const FrameWrapper::Frame& frame)
	MOCK_METHOD(Converters::FrameToMatConverter, Convert, cv::Mat, (frame) )

void FrameToMatConverter::Convert(const FrameWrapper::Frame& frame, cv::Mat& conversion)
	MOCK_VOID_METHOD(Converters::FrameToMatConverter, Convert, cv::Mat, (frame), (conversion) )	
}
/** @} */
