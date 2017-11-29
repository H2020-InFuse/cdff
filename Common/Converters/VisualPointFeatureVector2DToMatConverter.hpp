/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector2DToMatConverter.hpp
 * @date 28/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Types
 * 
 *  This is the class for type conversion from VisualPointFeatureVector2D to Mat.
 *  
 *
 * @{
 */

#ifndef VISUAL_POINT_FEATURE_2D_VECTOR_TO_MAT_CONVERTER_HPP
#define VISUAL_POINT_FEATURE_2D_VECTOR_TO_MAT_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <VisualPointFeatureVector2D.h>
#include <opencv2/core/core.hpp>


namespace Types {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class VisualPointFeatureVector2DToMatConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual cv::Mat Convert(VisualPointFeatureVector2D* vector);

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

/* VisualPointFeatureVector2DToMatConverter.hpp */
/** @} */
