/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file EigenTransformToTransform3DConverter.cpp
 * @date 22/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * Implementation of EigenTransformToTransform3DConverter.
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

#include "EigenTransformToTransform3DConverter.hpp"
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
EigenTransformToTransform3DConverter::~EigenTransformToTransform3DConverter()
	{

	}

const PoseWrapper::Transform3DConstPtr EigenTransformToTransform3DConverter::Convert(const Eigen::Matrix4f& transform)
	MOCK_METHOD(Converters::EigenTransformToTransform3DConverter, Convert, PoseWrapper::Transform3DConstPtr, (transform) )
	
}
/** @} */
