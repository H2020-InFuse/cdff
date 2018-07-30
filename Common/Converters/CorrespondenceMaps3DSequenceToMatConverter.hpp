/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file CorrespondenceMaps3DSequenceToMatConverter.hpp
 * @date 24/07/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from CorrespondenceMaps3D to a measurement opencv matrix.
 *  
 *
 * @{
 */

#ifndef CORRESPONDENCE_MAPS_3D_SEQUENCE_TO_MAT_CONVERTER
#define CORRESPONDENCE_MAPS_3D_SEQUENCE_TO_MAT_CONVERTER


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
class CorrespondenceMaps3DSequenceToMatConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual const cv::Mat Convert(const CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequenceConstPtr& correspondenceMapsSequence);
		const cv::Mat ConvertShared(const CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequenceSharedConstPtr& correspondenceMapsSequence);

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
		struct CloudPoint
		{
			float x;
			float y;
			float z;
			int cloud;
			bool explored;
			CloudPoint(float initX, float initY, float initZ, int initCloud, bool initExplored)
				{ x = initX; y = initY; z = initZ; cloud = initCloud; explored = initExplored; }
		};

		const cv::Mat ComputeMeasurementMatrix(const CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequence& correspondenceMapsSequence);
		std::vector<CloudPoint> ComputeChainOfMatchingPoints(
			const CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequence& correspondenceMapsSequence, int numberOfClouds, CloudPoint point1, CloudPoint point2);
		void AddChainToMeasurementMatrix(const std::vector<CloudPoint> chain, cv::Mat& measurementMatrix);
		int ComputeNumberOfClouds(const CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequence& correspondenceMapsSequence); 

		bool ThereAreUnexploredPoints(const std::vector<CloudPoint>& chain, int& chainIndexToExplore);
		std::vector<int> ComputeMapsToExplore(int numberOfClouds, int cloudIndex, int& separationPoint);
		int GetSourceIndex(int numberOfClouds, int mapIndex, int cloudIndex);
		int GetSinkIndex(int numberOfClouds, int mapIndex, int cloudIndex);
		bool AllCloudsAreRepresented(const std::vector<CloudPoint>& chain, int numberOfClouds);
		bool CloudIsNotInChain(const std::vector<CloudPoint>& chain, int cloudIndex);
		bool PointIsNotInVector(BaseTypesWrapper::Point3D point, const std::vector<BaseTypesWrapper::Point3D>& vector);
	};

}

#endif

/* CorrespondenceMaps2DSequenceToMatConverter.hpp */
/** @} */
