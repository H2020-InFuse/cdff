/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file EigenTransformToTransform3DConverter.hpp
 * @date 22/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from Eigen Transform to ASN Transform.
 *  
 *
 * @{
 */

#ifndef EIGEN_TRANSFORM_TO_TRANSFORM_3D_CONVERTER_HPP
#define EIGEN_TRANSFORM_TO_TRANSFORM_3D_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Pose.hpp>
#include <Eigen/Core>

namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class EigenTransformToTransform3DConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual const PoseWrapper::Transform3DConstPtr Convert(const Eigen::Matrix4f& transform);
		const PoseWrapper::Transform3DSharedConstPtr ConvertShared(const Eigen::Matrix4f& transform);

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

/* EigenTransformToTransform3DConverter.hpp */
/** @} */
