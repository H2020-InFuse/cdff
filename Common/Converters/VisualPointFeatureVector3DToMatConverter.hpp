/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector3DToMatConverter.hpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from VisualPointFeatureVector3D to Mat.
 *  
 *
 * @{
 */

#ifndef VISUAL_POINT_FEATURE_3D_VECTOR_TO_MAT_CONVERTER_HPP
#define VISUAL_POINT_FEATURE_3D_VECTOR_TO_MAT_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <VisualPointFeatureVector3D.hpp>
#include <opencv2/core/core.hpp>


namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class VisualPointFeatureVector3DToMatConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual const cv::Mat Convert(CppTypes::VisualPointFeatureVector3D::ConstPtr featuresVector);

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

/* VisualPointFeatureVector3DToMatConverter.hpp */
/** @} */
