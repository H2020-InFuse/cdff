/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file MultipleCorrespondences2DRecorder.hpp
 * @date 13/08/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * This is the storage of correspondence 2D Maps, let (L0, R0), (L1, R1), ..., (LN, RN) be a sequence of image pair from the most recent to the oldest.
 * The correspondences between images are stored in the following order (L0-R0), (L0-L1), (L0-R1), ..., (L0-RN), (R0-L0), (R0-L1), (R0-R1), ..., 
 * (R0, LN), (L1-R1), (L1-L2), ..., (L1-RN), ...., (LN-RN). The number N is defined by the parameter numberOfAdjustedStereoPairs.
 * Only the most recent N image pairs are kept in storage, the others will be discarded.
 * 
 * @{
 */

#ifndef MULTIPLECORRESPONDENCESRECORDER_HPP
#define MULTIPLECORRESPONDENCESRECORDER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "CorrespondenceMaps2DSequence.hpp"
#include "CorrespondenceMap3D.hpp"
#include "CorrespondenceMap2D.hpp"
#include "PointCloud.hpp"


namespace dfpc_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class MultipleCorrespondences2DRecorder
	{
	public:
		MultipleCorrespondences2DRecorder(int maximumNumberOfPoses);
		~MultipleCorrespondences2DRecorder();

		void InitializeNewSequence();
		void AddCorrespondencesFromOneImagePair(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr map);
		void AddCorrespondencesFromTwoImagePairs(std::vector<CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr> mapList);
		void CompleteNewSequence();
		
		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequencePtr GetLatestCorrespondences();
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
		int numberOfTemporaryRightMaps;

		WorkingSequence latestSequence;
		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequencePtr firstCorrespondenceMapSequence;
		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequencePtr secondCorrespondenceMapSequence;

		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequencePtr workingCorrespondenceMapSequence;
		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequencePtr historyCorrespondenceMapSequence;

		CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr temporaryLeftPastRightMap;
		std::vector<CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr> temporaryRightMaps;
	
	};

}
#endif
/* MultipleCorrespondences2DRecorder.hpp */
/** @} */
