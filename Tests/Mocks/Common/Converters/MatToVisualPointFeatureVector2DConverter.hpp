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
 * This is a mock for the converter from cv matrix to MatToVisualPointFeatureVector2D
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
#include <Converters/MatToVisualPointFeatureVector2DConverter.hpp>


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
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr Convert(const cv::Mat& featuresMatrix) override;

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
