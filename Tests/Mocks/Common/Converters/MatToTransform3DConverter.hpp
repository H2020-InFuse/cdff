/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToTransform3DConverter.hpp
 * @date 20/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * This is a mock for the converter from Mat to Transform 3D
 * 
 * 
 * @{
 */

#ifndef MOCKS_MAT_TO_TRANSFORM_3D_CONVERTER_HPP
#define MOCKS_MAT_TO_TRANSFORM_3D_CONVERTER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Mocks/Mock.hpp"
#include <MatToTransform3DConverter.hpp>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class MatToTransform3DConverter : public Mock, public Converters::MatToTransform3DConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual ~MatToTransform3DConverter();
		const PoseWrapper::Transform3DConstPtr Convert(const cv::Mat transform);

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

typedef MatToTransform3DConverter MatToPose3DConverter;


}

#endif

/* MatToTransform3DConverter.hpp */
/** @} */
