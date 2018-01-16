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
 * @addtogroup Converters
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
#include <VisualPointFeatureVector2D.hpp>
#include <opencv2/core/core.hpp>


namespace Converters {

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
		virtual VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr Convert(const cv::Mat& featuresMatrix);
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DSharedConstPtr ConvertShared(const cv::Mat& featuresMatrix);

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
