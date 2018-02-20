/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file Transform3DToMatConverter.hpp
 * @date 20/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from ASN Transform to Mat.
 *  
 *
 * @{
 */

#ifndef TRANSFORM_3D_TO_MAT_CONVERTER_HPP
#define TRANSFORM_3D_TO_MAT_CONVERTER_HPP


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
class Transform3DToMatConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual const cv::Mat Convert(const PoseWrapper::Transform3DConstPtr& transform);
		const cv::Mat ConvertShared(const PoseWrapper::Transform3DSharedConstPtr& transform);

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
