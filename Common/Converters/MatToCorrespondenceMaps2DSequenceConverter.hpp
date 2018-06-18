/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file MatToCorrespondenceMaps2DSequenceConverter.hpp
 * @date 18/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from Measurement opencv matrix to CorrespondenceMapsSequence.
 *  
 *
 * @{
 */

#ifndef MAT_TO_CORRESPONDENCE_MAPS_2D_SEQUENCE_CONVERTER
#define MAT_TO_CORRESPONDENCE_MAPS_2D_SEQUENCE_CONVERTER


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <CorrespondenceMaps2DSequence.hpp>
#include <opencv2/core/core.hpp>

namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class MatToCorrespondenceMaps2DSequenceConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr Convert(const cv::Mat&  frame);
		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceSharedConstPtr ConvertShared(const cv::Mat&  frame);

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
		static const int MAXIMUM_NUMBER_OF_IMAGES = 8;

	};

}

#endif

/* MatToCorrespondenceMaps2DSequenceConverter.hpp */
/** @} */
