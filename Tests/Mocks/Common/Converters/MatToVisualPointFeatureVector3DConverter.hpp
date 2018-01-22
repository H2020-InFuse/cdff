/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToVisualPointFeatureVector3DConverter.hpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * This is a mock for the converter from cv Matrix to VisualPointFeatureVector 3D
 * 
 * 
 * @{
 */

#ifndef MOCKS_MAT_TO_VISUAL_POINT_FEATURE_VECTOR_3D_CONVERTER_HPP
#define MOCKS_MAT_TO_VISUAL_POINT_FEATURE_VECTOR_3D_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Mocks/Mock.hpp"
#include <MatToVisualPointFeatureVector3DConverter.hpp>


namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class MatToVisualPointFeatureVector3DConverter : public Mock, public Converters::MatToVisualPointFeatureVector3DConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual ~MatToVisualPointFeatureVector3DConverter();
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr Convert(const cv::Mat& featuresMatrix);

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

/* MatToVisualPointFeatureVector3DConverter.hpp */
/** @} */
