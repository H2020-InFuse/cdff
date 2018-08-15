/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file MultipleCorrespondencesRecorder.hpp
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

#ifndef MULTIPLECORRESPONDENCESRECORDER_HPP
#define MULTIPLECORRESPONDENCESRECORDER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "CorrespondenceMaps3DSequence.hpp"
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
class MultipleCorrespondencesRecorder
	{
	public:
		MultipleCorrespondencesRecorder(int maximumNumberOfPoses);
		~MultipleCorrespondencesRecorder();

		void InitializeNewSequence();
		void AddCorrespondencesFromTwoImagePairs(std::vector<CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr> mapList,
			std::vector<PointCloudWrapper::PointCloudConstPtr> pointCloudList);
		void CompleteNewSequence();
		
		CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequencePtr GetLatestCorrespondences();
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
	
	};

}
#endif
/* MultipleCorrespondencesRecorder.hpp */
/** @} */
