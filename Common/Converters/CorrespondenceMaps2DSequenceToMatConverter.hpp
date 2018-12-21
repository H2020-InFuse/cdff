/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file CorrespondenceMaps2DSequenceToMatConverter.hpp
 * @date 18/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from CorrespondenceMaps2D to a measurement opencv matrix.
 *  
 *
 * @{
 */

#ifndef CORRESPONDENCE_MAPS_2D_SEQUENCE_TO_MAT_CONVERTER
#define CORRESPONDENCE_MAPS_2D_SEQUENCE_TO_MAT_CONVERTER


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Types/CPP/CorrespondenceMaps2DSequence.hpp>
#include <opencv2/core/core.hpp>

namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class CorrespondenceMaps2DSequenceToMatConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual const cv::Mat Convert(const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr& correspondenceMapsSequence);
		const cv::Mat ConvertShared(const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceSharedConstPtr& correspondenceMapsSequence);

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
		struct ImagePoint
		{
			int x;
			int y;
			int image;
			bool explored;
			ImagePoint(float initX, float initY, int initImage, bool initExplored)
				{ x = initX; y = initY; image = initImage; explored = initExplored; }
		};

		const cv::Mat ComputeMeasurementMatrix(const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence);
		std::vector<ImagePoint> ComputeChainOfMatchingPoints(
			const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence, int numberOfImages, ImagePoint point1, ImagePoint point2);
		void AddChainToMeasurementMatrix(const std::vector<ImagePoint>& chain, cv::Mat& measurementMatrix);
		int ComputeNumberOfImages(const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence); 

		bool ThereAreUnexploredPoints(const std::vector<ImagePoint>& chain, int& chainIndexToExplore);
		std::vector<int> ComputeMapsToExplore(int numberOfImages, int imageIndex, int& separationPoint);
		int GetSourceIndex(int numberOfImages, int mapIndex, int imageIndex);
		int GetSinkIndex(int numberOfImages, int mapIndex, int imageIndex);
		bool AllImagesAreRepresented(const std::vector<ImagePoint>& chain, int numberOfImages);
		bool ImageIsNotInChain(const std::vector<ImagePoint>& chain, int imageIndex);
		bool PointIsNotInVector(BaseTypesWrapper::Point2D point, const std::vector<BaseTypesWrapper::Point2D>& vector);
	};

}

#endif

/* CorrespondenceMaps2DSequenceToMatConverter.hpp */
/** @} */
