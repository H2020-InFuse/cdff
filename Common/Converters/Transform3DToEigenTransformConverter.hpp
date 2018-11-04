/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file Transform3DToEigenTransformConverter.hpp
 * @date 22/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from ASN Transform to Eigen Transform.
 *  
 *
 * @{
 */

#ifndef TRANSFORM_3D_TO_EIGEN_TRANSFORM_CONVERTER_HPP
#define TRANSFORM_3D_TO_EIGEN_TRANSFORM_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Types/CPP/Pose.hpp>
#include <Eigen/Core>


namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class Transform3DToEigenTransformConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual const Eigen::Matrix4f Convert(const PoseWrapper::Transform3DConstPtr& transform);
		const Eigen::Matrix4f ConvertShared(const PoseWrapper::Transform3DSharedConstPtr& transform);

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

/* Transform3DToEigenTransformConverter.hpp */
/** @} */
