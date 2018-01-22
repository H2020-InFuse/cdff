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
 * @addtogroup Converters
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
#include <Eigen/Geometry>

namespace Converters {

using namespace PoseWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
const PoseWrapper::Transform3DConstPtr EigenTransformToTransform3DConverter::Convert(const Eigen::Matrix4f& transform)
	{
	ASSERT(transform(3,0) == 0 && transform(3,1) == 0 && transform(3,2) == 0 && transform(3,3) == 1, "EigenTransformToTransform3DConverter, An invalid transformation matrix was passed.")
	Eigen::Matrix3f eigenRotationMatrix = transform.block(0,0,3,3);
	ASSERT_CLOSE(std::abs(eigenRotationMatrix.determinant()), 1.00, 0.00001, "EigenTransformToTransform3DConverter, An invalid rotation was passed during conversion.")
	Eigen::Quaternionf eigenRotation (eigenRotationMatrix);

	Transform3DPtr conversion = new Transform3D();

	SetPosition(*conversion, transform(0, 3), transform(1, 3), transform(2, 3) );
	SetOrientation(*conversion, eigenRotation.x(), eigenRotation.y(), eigenRotation.z(), eigenRotation.w());

	return conversion;
	}

const PoseWrapper::Transform3DSharedConstPtr EigenTransformToTransform3DConverter::ConvertShared(const Eigen::Matrix4f& transform)
	{
	Transform3DConstPtr conversion = Convert(transform);
	Transform3DSharedConstPtr sharedConversion(conversion);
	return sharedConversion;
	} 

}

/** @} */
