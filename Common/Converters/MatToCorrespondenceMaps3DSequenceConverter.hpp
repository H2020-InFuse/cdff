/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file MatToCorrespondenceMaps3DSequenceConverter.hpp
 * @date 24/07/2018
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

#ifndef MAT_TO_CORRESPONDENCE_MAPS_3D_SEQUENCE_CONVERTER
#define MAT_TO_CORRESPONDENCE_MAPS_3D_SEQUENCE_CONVERTER


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <CorrespondenceMaps3DSequence.hpp>
#include <opencv2/core/core.hpp>

namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class MatToCorrespondenceMaps3DSequenceConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequenceConstPtr Convert(const cv::Mat&  measurementMatrix);
		CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequenceSharedConstPtr ConvertShared(const cv::Mat&  measurementMatrix);

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
		static const int MAXIMUM_NUMBER_OF_CLOUDS = 8;

	};

}

#endif

/* MatToCorrespondenceMaps3DSequenceConverter.hpp */
/** @} */
