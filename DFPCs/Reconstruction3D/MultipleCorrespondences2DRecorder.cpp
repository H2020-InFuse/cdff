#include "MultipleCorrespondences2DRecorder.hpp"
#include "Errors/Assert.hpp"
#include <cmath>

namespace dfpc_ci {

using namespace CorrespondenceMap2DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PointCloudWrapper;
using namespace BaseTypesWrapper;

MultipleCorrespondences2DRecorder::MultipleCorrespondences2DRecorder(int maximumNumberOfPoses) :
	MAXIMUM_NUMBER_OF_POSES(maximumNumberOfPoses)
	{
	firstCorrespondenceMapSequence = NewCorrespondenceMaps2DSequence();
	secondCorrespondenceMapSequence = NewCorrespondenceMaps2DSequence();
	latestSequence = WorkingSequence::SECOND_SEQUENCE;

	workingCorrespondenceMapSequence = firstCorrespondenceMapSequence;
	historyCorrespondenceMapSequence = secondCorrespondenceMapSequence;

	numberOfOldPoses = -1;
	addingNewSequence = false;
	oneCorrespondenceWasAddedSinceLastDiscard = false;
	}

MultipleCorrespondences2DRecorder::~MultipleCorrespondences2DRecorder()
	{
	delete(firstCorrespondenceMapSequence);
	delete(secondCorrespondenceMapSequence);
	}

#define MINIMUM(a,b) (a < b ? a : b)

void MultipleCorrespondences2DRecorder::InitializeNewSequence()
	{
	ASSERT(!addingNewSequence, "InitializeNewSequence, a sequence was never completed");
	if (latestSequence == WorkingSequence::SECOND_SEQUENCE)
		{
		latestSequence = WorkingSequence::FIRST_SEQUENCE;
		}
	else
		{
		latestSequence = WorkingSequence::SECOND_SEQUENCE;
		}

	if (latestSequence == WorkingSequence::FIRST_SEQUENCE)
		{
		workingCorrespondenceMapSequence = firstCorrespondenceMapSequence;
		historyCorrespondenceMapSequence = secondCorrespondenceMapSequence;
		}
	else
		{
		workingCorrespondenceMapSequence = secondCorrespondenceMapSequence;
		historyCorrespondenceMapSequence = firstCorrespondenceMapSequence;
		}
	Clear(*workingCorrespondenceMapSequence);

	if (numberOfOldPoses <= MAXIMUM_NUMBER_OF_POSES) // When numberOfOldPoses is equal to MAXIMUM_NUMBER_OF_POSES+1, it actually means any number beyond MAXIMUM_NUMBER_OF_POSES.
		{
		numberOfOldPoses++;
		}

	addingNewSequence = true;
	expectedMapsToAdd = 4*MINIMUM(numberOfOldPoses, MAXIMUM_NUMBER_OF_POSES-1)+1;
	}

void MultipleCorrespondences2DRecorder::AddCorrespondences(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr map)
	{
	int addedMaps = GetNumberOfCorrespondenceMaps(*workingCorrespondenceMapSequence);
	ASSERT(addedMaps < expectedMapsToAdd, "AddCorrespondences, Unexpected number of maps in sequence");
	AddCorrespondenceMap(*workingCorrespondenceMapSequence, *map);
	}

void MultipleCorrespondences2DRecorder::CompleteNewSequence()
	{
	ASSERT(addingNewSequence, "CompleteNewSequence, method  InitializeNewSequence was not called before");
	int addedMaps = GetNumberOfCorrespondenceMaps(*workingCorrespondenceMapSequence);
	ASSERT( (addedMaps == expectedMapsToAdd), "CompleteNewSequence, Unexpected number of maps in sequence");

	if (numberOfOldPoses < MAXIMUM_NUMBER_OF_POSES)
		{
		for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondenceMaps(*historyCorrespondenceMapSequence); correspondenceIndex++)
			{
			AddCorrespondenceMap(*workingCorrespondenceMapSequence, GetCorrespondenceMap(*historyCorrespondenceMapSequence, correspondenceIndex));
			}
		}
	else
		{
		int correspondenceIndex = 0;
		for(int sourceIndex = 0; sourceIndex < 2*(MAXIMUM_NUMBER_OF_POSES-1); sourceIndex++)
			{
			for(int sinkIndex = sourceIndex + 1; sinkIndex < 2*(MAXIMUM_NUMBER_OF_POSES-1); sinkIndex++)
				{
				AddCorrespondenceMap(*workingCorrespondenceMapSequence, GetCorrespondenceMap(*historyCorrespondenceMapSequence, correspondenceIndex));	
				correspondenceIndex++;
				}
			correspondenceIndex += 2;
			}
		}

	addingNewSequence = false;
	oneCorrespondenceWasAddedSinceLastDiscard = true;
	}

CorrespondenceMaps2DSequencePtr MultipleCorrespondences2DRecorder::GetLatestCorrespondences()
	{
	if (oneCorrespondenceWasAddedSinceLastDiscard)
		{
		return workingCorrespondenceMapSequence;
		}
	else
		{
		return historyCorrespondenceMapSequence;
		}
	}

void MultipleCorrespondences2DRecorder::DiscardLatestCorrespondences()
	{
	ASSERT(oneCorrespondenceWasAddedSinceLastDiscard, "MultipleCorrespondences2DRecorder: Cannot discard the oldest correspondences. Only two correspondences are saved.!");
	if (latestSequence == WorkingSequence::SECOND_SEQUENCE)
		{
		latestSequence = WorkingSequence::FIRST_SEQUENCE;
		}
	else
		{
		latestSequence = WorkingSequence::SECOND_SEQUENCE;
		}

	if (numberOfOldPoses >= 0)
		{
		numberOfOldPoses--;
		}
	oneCorrespondenceWasAddedSinceLastDiscard = false;
	}

}
/** @} */
