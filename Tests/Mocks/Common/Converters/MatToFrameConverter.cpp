/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToFrameConverter.cpp
 * @date 19/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * Implementation of MatToFrameConverter.
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
#include <Mocks/MockMacro.hpp>

namespace Mocks {

using namespace FrameWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
MatToFrameConverter::~MatToFrameConverter()
	{

	}

const FrameConstPtr MatToFrameConverter::Convert(const cv::Mat frame)
	MOCK_METHOD(Converters::MatToFrameConverter, Convert, FrameConstPtr, (frame) )
	
}
/** @} */
