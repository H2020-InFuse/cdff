#include "MultipleCorrespondences2DRecorder.hpp"
#include "Errors/Assert.hpp"
#include <cmath>

namespace CDFF
{
namespace DFPC
{
namespace Reconstruction3D
{

using namespace CorrespondenceMap2DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PointCloudWrapper;
using namespace BaseTypesWrapper;

MultipleCorrespondences2DRecorder::MultipleCorrespondences2DRecorder(int maximumNumberOfPoses, bool filterPointsThatDoNotAppearInAllMatches) :
	MAXIMUM_NUMBER_OF_POSES(maximumNumberOfPoses),
	MAXIMUM_NUMBER_OF_MAPS( maximumNumberOfPoses * (maximumNumberOfPoses-1) / 2 )
	{
	firstCorrespondenceMapSequence = NewCorrespondenceMaps2DSequence();
	secondCorrespondenceMapSequence = NewCorrespondenceMaps2DSequence();
	latestSequence = WorkingSequence::SECOND_SEQUENCE;

	workingCorrespondenceMapSequence = firstCorrespondenceMapSequence;
	historyCorrespondenceMapSequence = secondCorrespondenceMapSequence;

	numberOfOldPoses = -1;
	addingNewSequence = false;
	oneCorrespondenceWasAddedSinceLastDiscard = false;
	useFilter = filterPointsThatDoNotAppearInAllMatches;
	expectedMapsToAdd = 0;

	if (useFilter)
		{
		filteredCorrespondenceMapSequence = NewCorrespondenceMaps2DSequence();
		}
	}

MultipleCorrespondences2DRecorder::~MultipleCorrespondences2DRecorder()
	{
	delete(firstCorrespondenceMapSequence);
	delete(secondCorrespondenceMapSequence);
	if (useFilter)
		{
		delete(filteredCorrespondenceMapSequence);
		}
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

CorrespondenceMaps2DSequenceConstPtr MultipleCorrespondences2DRecorder::GetLatestCorrespondences()
	{
	if (oneCorrespondenceWasAddedSinceLastDiscard)
		{
		if (useFilter)
			{
			return Filter(workingCorrespondenceMapSequence);
			}
		else
			{
			return workingCorrespondenceMapSequence;
			}
		}
	else
		{
		if (useFilter)
			{
			return Filter(historyCorrespondenceMapSequence);
			}
		else
			{
			return historyCorrespondenceMapSequence;
			}
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

CorrespondenceMaps2DSequenceConstPtr MultipleCorrespondences2DRecorder::Filter(CorrespondenceMaps2DSequenceConstPtr sequenceToFilter)
	{
	ASSERT(!addingNewSequence, "MultipleCorrespondences2DRecorder::Filter, you can call the method only when you are not adding new sequences");
	Copy(*sequenceToFilter, *filteredCorrespondenceMapSequence);

	int numberOfMaps = GetNumberOfCorrespondenceMaps(*filteredCorrespondenceMapSequence);
	if ( numberOfMaps < MAXIMUM_NUMBER_OF_MAPS )
		{
		return filteredCorrespondenceMapSequence;
		}

	std::vector < std::vector<BaseTypesWrapper::T_UInt32> > validChains;
	const CorrespondenceMap2D& firstMap = GetCorrespondenceMap(*filteredCorrespondenceMapSequence, 0);
	for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(firstMap); correspondenceIndex++)
		{
		std::vector<BaseTypesWrapper::T_UInt32> newChain = ComputeChainFrom( correspondenceIndex );
		if (newChain.size() == numberOfMaps)
			{
			validChains.push_back( newChain );
			} 
		}

	for(int mapIndex = 0; mapIndex < numberOfMaps; mapIndex++)
		{
		const CorrespondenceMap2D& map = GetCorrespondenceMap(*filteredCorrespondenceMapSequence, mapIndex);
		std::vector<BaseTypesWrapper::T_UInt32> removeIndexList;
		for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(map); correspondenceIndex++)
			{
			bool found = false;
			for(int chainIndex = 0; chainIndex < validChains.size() && !found; chainIndex++)
				{
				found = (validChains.at(chainIndex).at(mapIndex) == correspondenceIndex );
				}
			if (!found)
				{
				removeIndexList.push_back(correspondenceIndex);
				}
			}
		RemoveCorrespondences(*filteredCorrespondenceMapSequence, mapIndex, removeIndexList);
		}

	return filteredCorrespondenceMapSequence;
	}

std::vector<BaseTypesWrapper::T_UInt32> MultipleCorrespondences2DRecorder::ComputeChainFrom(int correspondenceIndex)
	{
	std::vector<BaseTypesWrapper::T_UInt32> chain;
	chain.push_back(correspondenceIndex);

	int mapIndex = 0;
	int lastAddedMapBySink = 0;
	int lastAddedIndexBySink = correspondenceIndex;
	int lastAddedIndex = correspondenceIndex;
	for(int sourceIndex = 0; sourceIndex < (2*MAXIMUM_NUMBER_OF_POSES - 1); sourceIndex++)
		{	
		int startSink = (sourceIndex == 0 ? 2 : sourceIndex+1);
		for(int sinkIndex = startSink; sinkIndex < 2*MAXIMUM_NUMBER_OF_POSES; sinkIndex++)
			{
			mapIndex++;
			int index;
			if ( sourceIndex > 0 && sinkIndex == sourceIndex + 1)
				{
				index = GetPointConnectedSinkToSource(lastAddedMapBySink, lastAddedIndexBySink, mapIndex);
				}
			else
				{
				index = GetPointConnectedSourceToSource(mapIndex-1, lastAddedIndex, mapIndex);
				}
			if (index == -1)
				{
				return std::vector<BaseTypesWrapper::T_UInt32>();
				}
			chain.push_back(index);
			if ( !LastSinkIsValid(chain, sourceIndex) )
				{
				return std::vector<BaseTypesWrapper::T_UInt32>();
				}
			lastAddedIndex = index;
			if ( sourceIndex > 0 && sinkIndex == sourceIndex + 1)
				{
				lastAddedMapBySink = mapIndex;
				lastAddedIndexBySink = index;
				}
			}
		}
	return chain;
	}

int MultipleCorrespondences2DRecorder::GetPointConnectedSourceToSource(int mapIndex1, int correspondenceIndex, int mapIndex2)
	{
	const CorrespondenceMap2D& map1 = GetCorrespondenceMap(*filteredCorrespondenceMapSequence, mapIndex1);
	const CorrespondenceMap2D& map2 = GetCorrespondenceMap(*filteredCorrespondenceMapSequence, mapIndex2);

	BaseTypesWrapper::Point2D sourcePoint1 = GetSource(map1, correspondenceIndex);

	for(int index = 0; index < GetNumberOfCorrespondences(map2); index++)
		{
		BaseTypesWrapper::Point2D sourcePoint2 = GetSource(map2, index);
		if (sourcePoint2.x == sourcePoint1.x && sourcePoint2.y == sourcePoint1.y)
			{
			return index;
			}
		}
	return -1;
	}

int MultipleCorrespondences2DRecorder::GetPointConnectedSinkToSource(int mapIndex1, int correspondenceIndex, int mapIndex2)
	{
	const CorrespondenceMap2D& map1 = GetCorrespondenceMap(*filteredCorrespondenceMapSequence, mapIndex1);
	const CorrespondenceMap2D& map2 = GetCorrespondenceMap(*filteredCorrespondenceMapSequence, mapIndex2);

	BaseTypesWrapper::Point2D sinkPoint1 = GetSink(map1, correspondenceIndex);

	for(int index = 0; index < GetNumberOfCorrespondences(map2); index++)
		{
		BaseTypesWrapper::Point2D sourcePoint2 = GetSource(map2, index);
		if (sourcePoint2.x == sinkPoint1.x && sourcePoint2.y == sinkPoint1.y)
			{
			return index;
			}
		}
	return -1;
	}

bool MultipleCorrespondences2DRecorder::LastSinkIsValid(const std::vector<BaseTypesWrapper::T_UInt32>& chain, int sourceIndex)
	{
	if (chain.size() < 2*MAXIMUM_NUMBER_OF_POSES)
		{
		return true;
		}
	int mapIndex1 = chain.size()-1;
	int mapIndex2 = mapIndex1 - (2*MAXIMUM_NUMBER_OF_POSES - 1 - sourceIndex);

	const CorrespondenceMap2D& map1 = GetCorrespondenceMap(*filteredCorrespondenceMapSequence, mapIndex1);
	const CorrespondenceMap2D& map2 = GetCorrespondenceMap(*filteredCorrespondenceMapSequence, mapIndex2);

	BaseTypesWrapper::Point2D sinkPoint1 = GetSink(map1, chain.at(mapIndex1) );
	BaseTypesWrapper::Point2D sinkPoint2 = GetSink(map2, chain.at(mapIndex2) );

	return ( sinkPoint1.x == sinkPoint2.x && sinkPoint1.y == sinkPoint2.y);
	}

}
}
}
/** @} */
