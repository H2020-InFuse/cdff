/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup CorrespondenceMap2DWrapper
 * @{
 */

#include "CorrespondenceMap2D.hpp"
#include "Errors/AssertOnTest.hpp"

namespace CorrespondenceMap2DWrapper
{

using namespace BaseTypesWrapper;

void Copy(const CorrespondenceMap2D& source, CorrespondenceMap2D& destination)
{
	ClearCorrespondences(destination);
	int numberOfCorrespondences = GetNumberOfCorrespondences(source);
	for (int correspondenceIndex = 0; correspondenceIndex < numberOfCorrespondences; correspondenceIndex++)
	{
		AddCorrespondence(
			destination,
			GetSource(source, correspondenceIndex),
			GetSink(source, correspondenceIndex),
			GetProbability(source, correspondenceIndex)
		);
	}
}

CorrespondenceMap2DPtr NewCorrespondenceMap2D()
{
	CorrespondenceMap2DPtr correspondenceMap = new CorrespondenceMap2D();
	Initialize(*correspondenceMap);
	return correspondenceMap;
}

CorrespondenceMap2DSharedPtr NewSharedCorrespondenceMap2D()
{
	CorrespondenceMap2DSharedPtr sharedCorrespondenceMap = std::make_shared<CorrespondenceMap2D>();
	Initialize(*sharedCorrespondenceMap);
	return sharedCorrespondenceMap;
}

void Initialize(CorrespondenceMap2D& correspondenceMap)
{
	ClearCorrespondences(correspondenceMap);
}

void AddCorrespondence(CorrespondenceMap2D& correspondenceMap, Point2D source, Point2D sink, T_Float probability)
{
	ASSERT_ON_TEST(correspondenceMap.nCount < MAX_CORRESPONDENCES_2D, "Correspondence Map 3D maximum capacity has been reached");
	int currentIndex = correspondenceMap.nCount;
	correspondenceMap.arr[currentIndex].source.arr[0] = source.x;
	correspondenceMap.arr[currentIndex].source.arr[1] = source.y;
	correspondenceMap.arr[currentIndex].sink.arr[0] = sink.x;
	correspondenceMap.arr[currentIndex].sink.arr[1] = sink.y;
	correspondenceMap.arr[currentIndex].probability = probability;
	correspondenceMap.nCount++;
}

void ClearCorrespondences(CorrespondenceMap2D& correspondenceMap)
{
	correspondenceMap.nCount = 0;
}

int GetNumberOfCorrespondences(const CorrespondenceMap2D& correspondenceMap)
{
	return correspondenceMap.nCount;
}

Point2D GetSource(const CorrespondenceMap2D& correspondenceMap, const int correspondenceIndex)
{
	ASSERT_ON_TEST(correspondenceIndex < correspondenceMap.nCount, "A missing correspondence was requested from a correspondence map 3d");
	Point2D point;
	point.x = correspondenceMap.arr[correspondenceIndex].source.arr[0];
	point.y = correspondenceMap.arr[correspondenceIndex].source.arr[1];
	return point;
}

Point2D GetSink(const CorrespondenceMap2D& correspondenceMap, const int correspondenceIndex)
{
	ASSERT_ON_TEST(correspondenceIndex < correspondenceMap.nCount, "A missing correspondence was requested from a correspondence map 3d");
	Point2D point;
	point.x = correspondenceMap.arr[correspondenceIndex].sink.arr[0];
	point.y = correspondenceMap.arr[correspondenceIndex].sink.arr[1];
	return point;
}

T_Float GetProbability(const CorrespondenceMap2D& correspondenceMap, const int correspondenceIndex)
{
	ASSERT_ON_TEST(correspondenceIndex < correspondenceMap.nCount, "A missing correspondence was requested from a correspondence map 3d");
	return correspondenceMap.arr[correspondenceIndex].probability;
}

void RemoveCorrespondences(CorrespondenceMap2D& correspondenceMap, std::vector<BaseTypesWrapper::T_UInt32> correspondenceIndexOrderedList)
	{
	BaseTypesWrapper::T_UInt32 elementsToRemove = correspondenceIndexOrderedList.size();
	if ( elementsToRemove == 0)
		{
		return;
		}
	//Checking that input is ordered correctly.
	static const std::string errorMessage = "Remove Correspondences error, the second input was not an ORDERED list or some index is not within range";
	for(int listIndex = 1; listIndex < elementsToRemove-1; listIndex++)
		{
		ASSERT( correspondenceIndexOrderedList.at(listIndex-1) < correspondenceIndexOrderedList.at(listIndex), errorMessage);
		ASSERT(	correspondenceIndexOrderedList.at(listIndex) < correspondenceIndexOrderedList.at(listIndex+1), errorMessage);
		}
	ASSERT( correspondenceIndexOrderedList.at(elementsToRemove-1) < correspondenceMap.nCount, errorMessage);
	BaseTypesWrapper::T_UInt32 firstIndex = correspondenceIndexOrderedList.at(0);

	//Removing elements 
	BaseTypesWrapper::T_UInt32 nextIndexToRemove = 1;
	BaseTypesWrapper::T_UInt32 currentGap = 1;
	for(int correspondenceIndex = firstIndex; correspondenceIndex < correspondenceMap.nCount - elementsToRemove; correspondenceIndex++)
		{
		if (nextIndexToRemove < elementsToRemove && correspondenceIndex+currentGap == correspondenceIndexOrderedList.at(nextIndexToRemove))
			{
			currentGap++;
			nextIndexToRemove++;
			correspondenceIndex--; //This is to not allow the map index to step forward in the next iteration;
			}
		else
			{
			correspondenceMap.arr[correspondenceIndex].source = correspondenceMap.arr[correspondenceIndex+currentGap].source;
			correspondenceMap.arr[correspondenceIndex].sink = correspondenceMap.arr[correspondenceIndex+currentGap].sink;
			correspondenceMap.arr[correspondenceIndex].probability = correspondenceMap.arr[correspondenceIndex+currentGap].probability;
			}
		}
	correspondenceMap.nCount -= elementsToRemove;
	}

BitStream ConvertToBitStream(const CorrespondenceMap2D& map)
	{
	BaseTypesWrapper::ConvertToBitStream(map, asn1SccCorrespondenceMap2D_REQUIRED_BYTES_FOR_ENCODING, asn1SccCorrespondenceMap2D_Encode)
	}

void ConvertFromBitStream(BitStream bitStream, CorrespondenceMap2D& map)
	{
	BaseTypesWrapper::ConvertFromBitStream(bitStream, asn1SccCorrespondenceMap2D_REQUIRED_BYTES_FOR_ENCODING, map, asn1SccCorrespondenceMap2D_Decode)
	}
}

/** @} */
