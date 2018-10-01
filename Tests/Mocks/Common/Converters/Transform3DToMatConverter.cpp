/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Transform3DToMatConverter.cpp
 * @date 22/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * Implementation of Transform3DToMatConverter.
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

#include "Transform3DToMatConverter.hpp"
#include <Errors/Assert.hpp>
#include <Mocks/MockMacro.hpp>

namespace Mocks {

using namespace PoseWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
Transform3DToMatConverter::~Transform3DToMatConverter()
	{

	}

const cv::Mat Transform3DToMatConverter::Convert(const PoseWrapper::Transform3DConstPtr& transform)
	MOCK_METHOD(Converters::Transform3DToMatConverter, Convert, cv::Mat, (transform) )
	
}
/** @} */
