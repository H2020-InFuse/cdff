#include "MultipleCorrespondences3DRecorder.hpp"
#include "Errors/Assert.hpp"
#include <cmath>

namespace CDFF
{
namespace DFPC
{
namespace Reconstruction3D
{

using namespace CorrespondenceMap3DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PointCloudWrapper;
using namespace BaseTypesWrapper;

MultipleCorrespondences3DRecorder::MultipleCorrespondences3DRecorder(int maximumNumberOfPoses) :
	MAXIMUM_NUMBER_OF_POSES(maximumNumberOfPoses)
	{
	firstCorrespondenceMapSequence = NewCorrespondenceMaps3DSequence();
	secondCorrespondenceMapSequence = NewCorrespondenceMaps3DSequence();
	latestSequence = WorkingSequence::SECOND_SEQUENCE;

	workingCorrespondenceMapSequence = firstCorrespondenceMapSequence;
	historyCorrespondenceMapSequence = secondCorrespondenceMapSequence;

	temporaryMap = NewCorrespondenceMap3D();
	numberOfOldPoses = -1;
	addingNewSequence = false;
	oneCorrespondenceWasAddedSinceLastDiscard = false;
	numberOfMapsAddedSinceLastInitialization = 0;
	}

MultipleCorrespondences3DRecorder::~MultipleCorrespondences3DRecorder()
	{
	delete(firstCorrespondenceMapSequence);
	delete(secondCorrespondenceMapSequence);
	delete(temporaryMap);
	}

void MultipleCorrespondences3DRecorder::InitializeNewSequence()
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

void MultipleCorrespondences3DRecorder::AddCorrespondencesFromTwoImagePairs(std::vector<CorrespondenceMap2DConstPtr> mapList, std::vector<PointCloudWrapper::PointCloudConstPtr> pointCloudList)
	{
	ASSERT(addingNewSequence, "AddCorrespondencesFromTwoImagePairs, method  InitializeNewSequence was not called before");
	ASSERT( mapList.size() == 4 && pointCloudList.size() == 2, "EstimationFromStereo::Extract3DCorrespondencesFromTwoImagePairs, input do not have expected size");
	ClearCorrespondences(*temporaryMap);

	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = mapList.at(0);
	CorrespondenceMap2DConstPtr leftTimeCorrespondenceMap = mapList.at(1);
	CorrespondenceMap2DConstPtr rightTimeCorrespondenceMap = mapList.at(2);
	CorrespondenceMap2DConstPtr pastCorrespondenceMap = mapList.at(3);
	PointCloudConstPtr triangulatedKeypointCloud = pointCloudList.at(0);
	PointCloudConstPtr pastKeypointCloud = pointCloudList.at(1);
	for(int correspondenceIndex1 = 0; correspondenceIndex1 < GetNumberOfCorrespondences(*leftRightCorrespondenceMap); correspondenceIndex1++)
		{
		Point2D leftRightSource = GetSource(*leftRightCorrespondenceMap, correspondenceIndex1);
		Point2D leftRightSink = GetSink(*leftRightCorrespondenceMap, correspondenceIndex1);
		CONTINUE_ON_INVALID_2D_POINT(leftRightSource);
		CONTINUE_ON_INVALID_2D_POINT(leftRightSink);
		for(int correspondenceIndex2 = 0; correspondenceIndex2 < GetNumberOfCorrespondences(*leftTimeCorrespondenceMap); correspondenceIndex2++)
			{
			Point2D leftTimeSource = GetSource(*leftTimeCorrespondenceMap, correspondenceIndex2);
			Point2D leftTimeSink = GetSink(*leftTimeCorrespondenceMap, correspondenceIndex2);
			CONTINUE_ON_INVALID_2D_POINT(leftTimeSource);
			CONTINUE_ON_INVALID_2D_POINT(leftTimeSink);
			CONTINUE_ON_DISTINCT_POINTS(leftRightSource, leftTimeSource);
			for(int correspondenceIndex3 = 0; correspondenceIndex3 < GetNumberOfCorrespondences(*rightTimeCorrespondenceMap); correspondenceIndex3++)
				{
				Point2D rightTimeSource = GetSource(*rightTimeCorrespondenceMap, correspondenceIndex3);
				Point2D rightTimeSink = GetSink(*rightTimeCorrespondenceMap, correspondenceIndex3);
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

					Point3D sourcePoint, sinkPoint;
					sourcePoint.x = GetXCoordinate(*triangulatedKeypointCloud, correspondenceIndex1);
					sourcePoint.y = GetYCoordinate(*triangulatedKeypointCloud, correspondenceIndex1);
					sourcePoint.z = GetZCoordinate(*triangulatedKeypointCloud, correspondenceIndex1);
					sinkPoint.x = GetXCoordinate(*pastKeypointCloud, correspondenceIndex4);
					sinkPoint.y = GetYCoordinate(*pastKeypointCloud, correspondenceIndex4);
					sinkPoint.z = GetZCoordinate(*pastKeypointCloud, correspondenceIndex4);

					CONTINUE_ON_INVALID_3D_POINT(sourcePoint);
					CONTINUE_ON_INVALID_3D_POINT(sinkPoint);
					AddCorrespondence(*temporaryMap, sourcePoint, sinkPoint, 1);
					}
				}
			}
		}

	AddCorrespondenceMap(*workingCorrespondenceMapSequence, *temporaryMap);
	numberOfMapsAddedSinceLastInitialization++;
	}

#define MIN(a, b) ( a < b ? a : b )

void MultipleCorrespondences3DRecorder::CompleteNewSequence()
	{
	ASSERT(addingNewSequence, "AddCorrespondencesFromTwoImagePairs, method  InitializeNewSequence was not called before");
	ASSERT(numberOfMapsAddedSinceLastInitialization == MIN(numberOfOldPoses, MAXIMUM_NUMBER_OF_POSES-1), "Correspondences Recorder, complete new sequence, unexpected number of maps");

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
		for(int sourceIndex = 0; sourceIndex < MAXIMUM_NUMBER_OF_POSES-1; sourceIndex++)
			{
			for(int sinkIndex = sourceIndex + 1; sinkIndex < MAXIMUM_NUMBER_OF_POSES-1; sinkIndex++)
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

CorrespondenceMaps3DSequencePtr MultipleCorrespondences3DRecorder::GetLatestCorrespondences()
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

void MultipleCorrespondences3DRecorder::DiscardLatestCorrespondences()
	{
	ASSERT(oneCorrespondenceWasAddedSinceLastDiscard, "MultipleCorrespondences3DRecorder: Cannot discard the oldest correspondences. Only two correspondences are saved.!");
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
}
}
/** @} */
