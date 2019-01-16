/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup CorrespondenceMap3DWrapper
 * @{
 */

#include "CorrespondenceMap3D.hpp"
#include "Errors/AssertOnTest.hpp"
#include <Errors/Assert.hpp>

namespace CorrespondenceMap3DWrapper
{

using namespace BaseTypesWrapper;

void Copy(const CorrespondenceMap3D& source, CorrespondenceMap3D& destination)
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

CorrespondenceMap3DPtr NewCorrespondenceMap3D()
{
	CorrespondenceMap3DPtr correspondenceMap = new CorrespondenceMap3D();
	Initialize(*correspondenceMap);
	return correspondenceMap;
}

CorrespondenceMap3DSharedPtr NewSharedCorrespondenceMap3D()
{
	CorrespondenceMap3DSharedPtr sharedCorrespondenceMap = std::make_shared<CorrespondenceMap3D>();
	Initialize(*sharedCorrespondenceMap);
	return sharedCorrespondenceMap;
}

void Initialize(CorrespondenceMap3D& correspondenceMap)
{
	ClearCorrespondences(correspondenceMap);
}

void AddCorrespondence(CorrespondenceMap3D& correspondenceMap, Point3D source, Point3D sink, T_Float probability)
{
	ASSERT_ON_TEST(correspondenceMap.nCount < MAX_CORRESPONDENCES_3D, "Correspondence Map 3D maximum capacity has been reached");
	int currentIndex = correspondenceMap.nCount;
	correspondenceMap.arr[currentIndex].source.arr[0] = source.x;
	correspondenceMap.arr[currentIndex].source.arr[1] = source.y;
	correspondenceMap.arr[currentIndex].source.arr[2] = source.z;
	correspondenceMap.arr[currentIndex].sink.arr[0] = sink.x;
	correspondenceMap.arr[currentIndex].sink.arr[1] = sink.y;
	correspondenceMap.arr[currentIndex].sink.arr[2] = sink.z;
	correspondenceMap.arr[currentIndex].probability = probability;
	correspondenceMap.nCount++;
}

void ClearCorrespondences(CorrespondenceMap3D& correspondenceMap)
{
	correspondenceMap.nCount = 0;
}

int GetNumberOfCorrespondences(const CorrespondenceMap3D& correspondenceMap)
{
	return correspondenceMap.nCount;
}

Point3D GetSource(const CorrespondenceMap3D& correspondenceMap, const int correspondenceIndex)
{
	ASSERT_ON_TEST(correspondenceIndex < correspondenceMap.nCount, "A missing correspondence was requested from a correspondence map 3d");
	Point3D point;
	point.x = correspondenceMap.arr[correspondenceIndex].source.arr[0];
	point.y = correspondenceMap.arr[correspondenceIndex].source.arr[1];
	point.z = correspondenceMap.arr[correspondenceIndex].source.arr[2];
	return point;
}

Point3D GetSink(const CorrespondenceMap3D& correspondenceMap, const int correspondenceIndex)
{
	ASSERT_ON_TEST(correspondenceIndex < correspondenceMap.nCount, "A missing correspondence was requested from a correspondence map 3d");
	Point3D point;
	point.x = correspondenceMap.arr[correspondenceIndex].sink.arr[0];
	point.y = correspondenceMap.arr[correspondenceIndex].sink.arr[1];
	point.z = correspondenceMap.arr[correspondenceIndex].sink.arr[2];
	return point;
}

T_Float GetProbability(const CorrespondenceMap3D& correspondenceMap, const int correspondenceIndex)
{
	ASSERT_ON_TEST(correspondenceIndex < correspondenceMap.nCount, "A missing correspondence was requested from a correspondence map 3d");
	return correspondenceMap.arr[correspondenceIndex].probability;
}

void RemoveCorrespondences(CorrespondenceMap3D& correspondenceMap, std::vector<BaseTypesWrapper::T_UInt32> correspondenceIndexOrderedList)
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

BitStream ConvertToBitStream(const CorrespondenceMap3D& map)
	{
	return BaseTypesWrapper::ConvertToBitStream(map, asn1SccCorrespondenceMap3D_REQUIRED_BYTES_FOR_ENCODING, asn1SccCorrespondenceMap3D_Encode);
	}

void ConvertFromBitStream(BitStream bitStream, CorrespondenceMap3D& map)
	{
	BaseTypesWrapper::ConvertFromBitStream(bitStream, asn1SccCorrespondenceMap3D_REQUIRED_BYTES_FOR_ENCODING, map, asn1SccCorrespondenceMap3D_Decode);
	}
}

/** @} */
