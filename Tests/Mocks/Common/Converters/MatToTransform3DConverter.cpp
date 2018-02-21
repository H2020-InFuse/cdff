/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToTransform3DConverter.cpp
 * @date 20/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * Implementation of MatToTransform3DConverter.
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

#include "MatToTransform3DConverter.hpp"
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
MatToTransform3DConverter::~MatToTransform3DConverter()
	{

	}

const PoseWrapper::Transform3DConstPtr MatToTransform3DConverter::Convert(const cv::Mat transform)
	MOCK_METHOD(Converters::MatToTransform3DConverter, Convert, PoseWrapper::Transform3DConstPtr, (transform) )
	
}
/** @} */
