/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Transform3DToMatConverter.hpp
 * @date 22/02/2018
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

#ifndef MOCKS_TRANSFORM_3D_TO_MAT_CONVERTER_HPP
#define MOCKS_TRANSFORM_3D_TO_MAT_CONVERTER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Mocks/Mock.hpp"
#include <Converters/Transform3DToMatConverter.hpp>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class Transform3DToMatConverter : public Mock, public Converters::Transform3DToMatConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual ~Transform3DToMatConverter();
		const cv::Mat Convert(const PoseWrapper::Transform3DConstPtr& transform) override;

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

typedef Transform3DToMatConverter Pose3DToMatConverter;


}

#endif

/* Transform3DToMatConverter.hpp */
/** @} */
