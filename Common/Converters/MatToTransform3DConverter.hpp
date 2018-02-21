/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file MatToTransform3DConverter.hpp
 * @date 20/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from cv Matrix to ASN Transform.
 *  
 *
 * @{
 */

#ifndef MAT_TO_TRANSFORM_3D_CONVERTER_HPP
#define MAT_TO_TRANSFORM_3D_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Pose.hpp>
#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class MatToTransform3DConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual const PoseWrapper::Transform3DConstPtr Convert(const cv::Mat transform);
		const PoseWrapper::Transform3DSharedConstPtr ConvertShared(const cv::Mat transform);

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
