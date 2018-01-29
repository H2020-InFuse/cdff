/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector2DToMatConverter.hpp
 * @date 29/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * This is a mock for the converter from VisualPointFeatureVector2D to cv matrix
 * 
 * 
 * @{
 */

#ifndef MOCKS_VISUAL_POINT_FEATURE_VECTOR_2D_TO_MAT_CONVERTER_HPP
#define MOCKS_VISUAL_POINT_FEATURE_VECTOR_2D_TO_MAT_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Mocks/Mock.hpp"
#include <VisualPointFeatureVector2DToMatConverter.hpp>


namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class VisualPointFeatureVector2DToMatConverter : public Mock, public Converters::VisualPointFeatureVector2DToMatConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual ~VisualPointFeatureVector2DToMatConverter();
		const cv::Mat Convert(const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr& featuresVector);

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
