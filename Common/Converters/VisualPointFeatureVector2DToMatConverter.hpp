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
 * @addtogroup Converters
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
#include <Types/CPP/VisualPointFeatureVector2D.hpp>
#include <opencv2/core/core.hpp>


namespace Converters {

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
		virtual const cv::Mat Convert(const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr& vector);
		const cv::Mat ConvertShared(const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DSharedConstPtr& vector);

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
