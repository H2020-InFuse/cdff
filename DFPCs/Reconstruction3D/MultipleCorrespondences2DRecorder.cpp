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

	temporaryLeftPastRightMap = NewCorrespondenceMap2D();
	numberOfOldPoses = -1;
	addingNewSequence = false;
	oneCorrespondenceWasAddedSinceLastDiscard = false;
	numberOfTemporaryRightMaps = 0;

	for(int mapIndex = 0; mapIndex < 2* (MAXIMUM_NUMBER_OF_POSES - 1); mapIndex++)
		{
		temporaryRightMaps.push_back( NewCorrespondenceMap2D() );
		}
	}

MultipleCorrespondences2DRecorder::~MultipleCorrespondences2DRecorder()
	{
	delete(firstCorrespondenceMapSequence);
	delete(secondCorrespondenceMapSequence);
	delete(temporaryLeftPastRightMap);
	for(int mapIndex = 0; mapIndex < 2* (MAXIMUM_NUMBER_OF_POSES - 1); mapIndex++)
		{
		delete( temporaryRightMaps.at(mapIndex) );
		}
	}

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
	numberOfMapsAddedSinceLastInitialization = 0;
	numberOfTemporaryRightMaps = 0;
	}

#define CONTINUE_ON_INVALID_2D_POINT(point) \
	if (point.x != point.x || point.y != point.y) \
		{ \
		continue; \
		}
#define CONTINUE_ON_INVALID_3D_POINT(point) \
	if (point.x != point.x || point.y != point.y || point.z != point.z) \
		{ \
		continue; \
		}
#define CONTINUE_ON_DISTINCT_POINTS(point1, point2) \
	if (point1.x != point2.x || point1.y != point2.y) \
		{ \
		continue; \
		}

void MultipleCorrespondences2DRecorder::AddCorrespondencesFromOneImagePair(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr map)
	{
	ASSERT(numberOfOldPoses == 0, "You can call AddCorrespondencesFromOneImagePair, only the first time");
	ASSERT(numberOfMapsAddedSinceLastInitialization == 0, "You can call AddCorrespondencesFromOneImagePair only once");

	AddCorrespondenceMap(*workingCorrespondenceMapSequence, *map);
	numberOfMapsAddedSinceLastInitialization++;
	}

void MultipleCorrespondences2DRecorder::AddCorrespondencesFromTwoImagePairs(std::vector<CorrespondenceMap2DConstPtr> mapList)
	{
	ASSERT(addingNewSequence, "AddCorrespondencesFromTwoImagePairs, method  InitializeNewSequence was not called before");
	ASSERT( mapList.size() == 4, "EstimationFromStereo::Extract3DCorrespondencesFromTwoImagePairs, input do not have expected size");
	ASSERT( numberOfMapsAddedSinceLastInitialization < MAXIMUM_NUMBER_OF_POSES, "EstimationFromStereo::Extract3DCorrespondencesFromTwoImagePairs, sequence is too long");

	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = mapList.at(0);
	CorrespondenceMap2DConstPtr leftPastLeftCorrespondenceMap = mapList.at(1);
	CorrespondenceMap2DConstPtr rightPastRightCorrespondenceMap = mapList.at(2);
	CorrespondenceMap2DConstPtr pastCorrespondenceMap = mapList.at(3);

	//The left right correspondence will be repeated, it needs to be added only once.
	if (numberOfMapsAddedSinceLastInitialization == 0)
		{
		AddCorrespondenceMap(*workingCorrespondenceMapSequence, *leftRightCorrespondenceMap);
		}
	AddCorrespondenceMap(*workingCorrespondenceMapSequence, *leftPastLeftCorrespondenceMap);			

	ClearCorrespondences(*temporaryLeftPastRightMap);
	CorrespondenceMap2DPtr rightPastLeftMap = temporaryRightMaps.at(numberOfTemporaryRightMaps);
	ClearCorrespondences(*rightPastLeftMap);

	for(int correspondenceIndex1 = 0; correspondenceIndex1 < GetNumberOfCorrespondences(*leftRightCorrespondenceMap); correspondenceIndex1++)
		{
		Point2D leftRightSource = GetSource(*leftRightCorrespondenceMap, correspondenceIndex1);
		Point2D leftRightSink = GetSink(*leftRightCorrespondenceMap, correspondenceIndex1);
		CONTINUE_ON_INVALID_2D_POINT(leftRightSource);
		CONTINUE_ON_INVALID_2D_POINT(leftRightSink);
		for(int correspondenceIndex2 = 0; correspondenceIndex2 < GetNumberOfCorrespondences(*leftPastLeftCorrespondenceMap); correspondenceIndex2++)
			{
			Point2D leftTimeSource = GetSource(*leftPastLeftCorrespondenceMap, correspondenceIndex2);
			Point2D leftTimeSink = GetSink(*leftPastLeftCorrespondenceMap, correspondenceIndex2);
			CONTINUE_ON_INVALID_2D_POINT(leftTimeSource);
			CONTINUE_ON_INVALID_2D_POINT(leftTimeSink);
			CONTINUE_ON_DISTINCT_POINTS(leftRightSource, leftTimeSource);
			for(int correspondenceIndex3 = 0; correspondenceIndex3 < GetNumberOfCorrespondences(*rightPastRightCorrespondenceMap); correspondenceIndex3++)
				{
				Point2D rightTimeSource = GetSource(*rightPastRightCorrespondenceMap, correspondenceIndex3);
				Point2D rightTimeSink = GetSink(*rightPastRightCorrespondenceMap, correspondenceIndex3);
				CONTINUE_ON_INVALID_2D_POINT(rightTimeSource);
				CONTINUE_ON_INVALID_2D_POINT(rightTimeSink);
				CONTINUE_ON_DISTINCT_POINTS(leftRightSink, rightTimeSource);
				for(int correspondenceIndex4 = 0; correspondenceIndex4 < GetNumberOfCorrespondences(*pastCorrespondenceMap); correspondenceIndex4++)
					{
					Point2D pastSource = GetSource(*pastCorrespondenceMap, correspondenceIndex4);
					Point2D pastSink = GetSink(*pastCorrespondenceMap, correspondenceIndex4);
					CONTINUE_ON_INVALID_2D_POINT(pastSource);
					CONTINUE_ON_INVALID_2D_POINT(pastSink);
					CONTINUE_ON_DISTINCT_POINTS(pastSource, leftTimeSink);
					CONTINUE_ON_DISTINCT_POINTS(pastSink, rightTimeSink);

					AddCorrespondence(*temporaryLeftPastRightMap, leftRightSource, pastSink, 1);
					AddCorrespondence(*rightPastLeftMap, leftRightSink, pastSource, 1);
					}
				}
			}
		}
	AddCorrespondenceMap(*workingCorrespondenceMapSequence, *temporaryLeftPastRightMap);			

	CorrespondenceMap2DPtr rightPastRightMap = temporaryRightMaps.at(numberOfTemporaryRightMaps + 1);
	Copy(*rightPastRightCorrespondenceMap, *rightPastRightMap);

	numberOfTemporaryRightMaps += 2;
	numberOfMapsAddedSinceLastInitialization++;
	}

#define MIN(a, b) ( a < b ? a : b )

void MultipleCorrespondences2DRecorder::CompleteNewSequence()
	{
	ASSERT(addingNewSequence, "AddCorrespondencesFromTwoImagePairs, method  InitializeNewSequence was not called before");
	if (numberOfOldPoses == 0)
		{
		ASSERT( numberOfMapsAddedSinceLastInitialization == 1, "The first time you need to call AddCorrespondencesFromOneImagePair once");
		}
	else
		{
		ASSERT(numberOfMapsAddedSinceLastInitialization == MIN(numberOfOldPoses, MAXIMUM_NUMBER_OF_POSES-1), "Correspondences Recorder, complete new sequence, unexpected number of maps");
		}

	for(int mapIndex = 0; mapIndex < numberOfTemporaryRightMaps; mapIndex++)
		{
		AddCorrespondenceMap(*workingCorrespondenceMapSequence, *(temporaryRightMaps.at(mapIndex)) );			
		}

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
