/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file MultipleCorrespondences3DRecorder.hpp
 * @date 13/08/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 *
 * This is the storage of correspondence Maps, let (L0, R0), (L1, R1), ..., (LN, RN) be a sequence of image pair from the most recent to the oldest.
 * The correspondences between images are stored in the following order (L0-R0), (L0-L1), (L0-R1), ..., (L0-RN), (R0-L0), (R0-L1), (R0-R1), ...,
 * (R0, LN), (L1-R1), (L1-L2), ..., (L1-RN), ...., (LN-RN). The number N is defined by the parameter numberOfAdjustedStereoPairs.
 * Only the most recent N image pairs are kept in storage, the others will be discarded.
 *
 * @{
 */

#ifndef MultipleCorrespondences3DRecorder_HPP
#define MultipleCorrespondences3DRecorder_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Types/CPP/CorrespondenceMaps3DSequence.hpp>
#include <Types/CPP/CorrespondenceMap3D.hpp>
#include <Types/CPP/CorrespondenceMap2D.hpp>
#include <Types/CPP/PointCloud.hpp>


namespace CDFF
{
namespace DFPC
{
namespace Reconstruction3D
{

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * This class has the purpose of storing a sequence of 3D correspondence maps between image features. It offers:
 * (i) a mechanism for adding correspondence maps one by one;
 * (ii) a method for storing the latest two sequences of correspondence maps in an efficient way, and discard the most recent one if no longer useful.
 * --------------------------------------------------------------------------
 */
class MultipleCorrespondences3DRecorder
	{
	public:
		MultipleCorrespondences3DRecorder() = delete;
		explicit MultipleCorrespondences3DRecorder(int maximumNumberOfPoses);
		~MultipleCorrespondences3DRecorder();

		/* The following three methods define a protocol for adding a sequence of 2D correspondence maps:
		the call pattern is: [call to InitializeNewSequence] [call to AddCorrespondences zero or more times] [call to CompleteNewSequence]
		 */

		/*This method opens the sequence*/
		void InitializeNewSequence();

		/* This method adds only one corresspondence map (even if it takes as input a list):
		* 
		* @param mapList: this is a list of four 2D correspondence maps. The four correspondence maps represent correspondence between (i) left image A1 and right image A2
		* 	  of a stereo pairm (ii) A1 and left image B1 of another pair (B1-B2), (iii) A2 and right image B2, (iv) B1 and B2.
		* @param pointCloudList: this is a list of 2 point clouds: the point cloud extracted from the stereo pair A1-A2 and the point cloud extracted from the stereo pair B1-B2.
		* 
		* The correspondence map saved is a 3D point correspondence between the two point clouds as long as the point appears in all the 2d correspondence maps. 
		*/
		void AddCorrespondencesFromTwoImagePairs(std::vector<CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr> mapList,
			std::vector<PointCloudWrapper::PointCloudConstPtr> pointCloudList);

		/*This method closes the sequence*/
		void CompleteNewSequence();

		/* This method retrieves the sequence of correspondence maps that was previously inserted */
		CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequencePtr GetLatestCorrespondences();

		/*This method discards the sequence of correspondence maps that was previously inserted, and the older one becomes available for retrievial by the GetLatestCorrespondences() method
		This method will cause an error if it is called before adding a sequence or if it is called twice in a row without adding a sequence in between */
		void DiscardLatestCorrespondences();

	protected:

	private:

		enum class WorkingSequence : int
			{
			FIRST_SEQUENCE = 0,
			SECOND_SEQUENCE = 1,
			};
		const int MAXIMUM_NUMBER_OF_POSES;
		int numberOfOldPoses;
		bool addingNewSequence;
		bool oneCorrespondenceWasAddedSinceLastDiscard;
		int numberOfMapsAddedSinceLastInitialization;

		WorkingSequence latestSequence;
		CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequencePtr firstCorrespondenceMapSequence;
		CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequencePtr secondCorrespondenceMapSequence;

		CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequencePtr workingCorrespondenceMapSequence;
		CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequencePtr historyCorrespondenceMapSequence;

		CorrespondenceMap3DWrapper::CorrespondenceMap3DPtr temporaryMap;

		/*
		* Inline Methods
		*
		*/
		inline bool Point2DIsInvalid(const BaseTypesWrapper::Point2D& point)
			{
			return (point.x != point.x || point.y != point.y);
			}

		inline bool Point3DIsInvalid(const BaseTypesWrapper::Point3D& point)
			{
			return (point.x != point.x || point.y != point.y || point.z != point.z);
			}

		inline bool PointsAreDistinct(const BaseTypesWrapper::Point2D& point1, const BaseTypesWrapper::Point2D& point2)
			{
			return (point1.x != point2.x || point1.y != point2.y);
			}

	};

}
}
}
#endif
/* MultipleCorrespondences3DRecorder.hpp */
/** @} */
