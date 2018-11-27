/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file MatToVisualPointFeatureVector3DConverter.hpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from Mat to VisualPointFeatureVector3D.
 *  
 *
 * @{
 */

#ifndef MAT_TO_VISUAL_POINT_FEATURE_3D_VECTOR_CONVERTER_HPP
#define MAT_TO_VISUAL_POINT_FEATURE_3D_VECTOR_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <opencv2/core/core.hpp>


namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class MatToVisualPointFeatureVector3DConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr Convert(const cv::Mat& featuresMatrix);
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DSharedConstPtr ConvertShared(const cv::Mat& featuresMatrix);

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
