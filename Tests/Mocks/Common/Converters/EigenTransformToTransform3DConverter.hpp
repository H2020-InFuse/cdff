/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file EigenTransformToTransform3DConverter.hpp
 * @date 22/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * This is a mock for the converter from Eigen Transform to Transform 3D
 * 
 * 
 * @{
 */

#ifndef MOCKS_EIGEN_TRANSFORM_TO_TRANSFORM_3D_CONVERTER_HPP
#define MOCKS_EIGEN_TRANSFORM_TO_TRANSFORM_3D_CONVERTER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Mocks/Mock.hpp"
#include <Converters/EigenTransformToTransform3DConverter.hpp>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class EigenTransformToTransform3DConverter : public Mock, public Converters::EigenTransformToTransform3DConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual ~EigenTransformToTransform3DConverter();
		const PoseWrapper::Transform3DConstPtr Convert(const Eigen::Matrix4f& transform) override;

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
