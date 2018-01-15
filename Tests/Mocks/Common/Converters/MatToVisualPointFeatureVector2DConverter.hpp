/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToVisualPointFeatureVector2DConverter.hpp
 * @date 21/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * This is the main mocking class, it supports the programming of ad hoc behaviours. Every Mock should inherit from this class.
 * 
 * 
 * @{
 */

#ifndef MOCKS_MAT_TO_VISUAL_POINT_FEATURE_VECTOR_2D_CONVERTER_HPP
#define MOCKS_MAT_TO_VISUAL_POINT_FEATURE_VECTOR_2D_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Mocks/Mock.hpp"
#include <MatToVisualPointFeatureVector2DConverter.hpp>


namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class MatToVisualPointFeatureVector2DConverter : public Mock, public Converters::MatToVisualPointFeatureVector2DConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual ~MatToVisualPointFeatureVector2DConverter();
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr Convert(const cv::Mat& featuresVector);

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
