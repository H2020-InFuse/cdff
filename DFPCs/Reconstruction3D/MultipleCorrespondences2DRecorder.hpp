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
 * Only the Nmost recent image pairs are kept in storage, the others will be discarded.
 *
 * @{
 */

#ifndef MULTIPLECORRESPONDENCES2DRECORDER_HPP
#define MULTIPLECORRESPONDENCES2DRECORDER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Types/CPP/CorrespondenceMaps2DSequence.hpp>
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
 * This class has the purpose of storing a sequence of 2D correspondence maps between image features. It offers:
 * (i) a mechanism for adding correspondence maps one by one;
 * (ii) a method for storing the latest two sequences of correspondence maps in an efficient way, and discard the most recent one if no longer useful.
 * --------------------------------------------------------------------------
 */
class MultipleCorrespondences2DRecorder
	{
	public:
		MultipleCorrespondences2DRecorder(int maximumNumberOfPoses, bool filterPointsThatDoNotAppearInAllMatches = false);
		~MultipleCorrespondences2DRecorder();

		/* The following three methods define a protocol for adding a sequence of 2D correspondence maps:
		the call pattern is: [call to InitializeNewSequence] [call to AddCorrespondences zero or more times] [call to CompleteNewSequence]
		 */
		void InitializeNewSequence();
		void AddCorrespondences(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr map);
		void CompleteNewSequence();

		/* This method retrieves the sequence of correspondence maps that was previously inserted */
		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr GetLatestCorrespondences();

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
		const int MAXIMUM_NUMBER_OF_MAPS;
		int numberOfOldPoses;
		bool addingNewSequence;
		bool oneCorrespondenceWasAddedSinceLastDiscard;
		int expectedMapsToAdd;
		bool useFilter;

		WorkingSequence latestSequence;
		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequencePtr firstCorrespondenceMapSequence;
		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequencePtr secondCorrespondenceMapSequence;

		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequencePtr workingCorrespondenceMapSequence;
		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequencePtr historyCorrespondenceMapSequence;

		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequencePtr filteredCorrespondenceMapSequence;

		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr Filter(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr sequenceToFilter);
		std::vector<BaseTypesWrapper::T_UInt32> ComputeChainFrom(int correspondenceIndex);
		int GetPointConnectedSourceToSource(int mapIndex1, int correspondenceIndex, int mapIndex2);
		int GetPointConnectedSinkToSource(int mapIndex1, int correspondenceIndex, int mapIndex2);
		bool LastSinkIsValid(const std::vector<BaseTypesWrapper::T_UInt32>& chain, int sourceIndex);

	};

}
}
}
#endif
/* MultipleCorrespondences2DRecorder.hpp */
/** @} */
