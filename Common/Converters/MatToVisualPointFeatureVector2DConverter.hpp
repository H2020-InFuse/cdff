/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file MatToVisualPointFeatureVector2DConverter.hpp
 * @date 20/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Types
 * 
 *  This is the class for type conversion from Mat to VisualPointFeatureVector2D.
 *  
 *
 * @{
 */

#ifndef MAT_TO_VISUAL_POINT_FEATURE_2D_VECTOR_CONVERTER_HPP
#define MAT_TO_VISUAL_POINT_FEATURE_2D_VECTOR_CONVERTER_HPP


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
class MatToVisualPointFeatureVector2DConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual VisualPointFeatureVector2D* Convert(cv::Mat image);

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

/* MatToVisualPointFeatureVector2DConverter.hpp */
/** @} */
