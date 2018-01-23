/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Transform3DToEigenTransformConverter.cpp
 * @date 22/01/2018
 * @authors Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of Transform3DToEigenTransformConverter class.
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

#include "Transform3DToEigenTransformConverter.hpp"
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
const Eigen::Matrix4f Transform3DToEigenTransformConverter::Convert(const PoseWrapper::Transform3DConstPtr& transform)
	{
	Eigen::Matrix4f conversion;

	Eigen::Quaternionf eigenRotation( GetWOrientation(*transform), GetXOrientation(*transform), GetYOrientation(*transform), GetZOrientation(*transform));
	Eigen::Matrix3f rotationMatrix = eigenRotation.toRotationMatrix();

	Eigen::Translation<float, 3> eigenTranslation( GetXPosition(*transform), GetYPosition(*transform), GetZPosition(*transform));	
	
	conversion << 	rotationMatrix(0,0), rotationMatrix(0,1), rotationMatrix(0,2), eigenTranslation.x(),
			rotationMatrix(1,0), rotationMatrix(1,1), rotationMatrix(1,2), eigenTranslation.y(),
			rotationMatrix(2,0), rotationMatrix(2,1), rotationMatrix(2,2), eigenTranslation.z(),
			0, 0, 0, 1;

	return conversion;
	}

const Eigen::Matrix4f Transform3DToEigenTransformConverter::ConvertShared(const PoseWrapper::Transform3DSharedConstPtr& transform)
	{
	return Convert(transform.get());
	}

}

/** @} */
