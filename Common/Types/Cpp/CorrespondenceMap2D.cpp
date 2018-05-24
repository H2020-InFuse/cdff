/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup CorrespondenceMap2DWrapper
 * @{
 */

#include "CorrespondenceMap2D.hpp"
#include <Errors/Assert.hpp>

namespace CorrespondenceMap2DWrapper
{

using namespace BaseTypesWrapper;

void Copy(const CorrespondenceMap2D& source, CorrespondenceMap2D& destination)
	{
	ClearCorrespondences(destination);
	for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(source); correspondenceIndex++)
		{
		AddCorrespondence
			(
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
	correspondenceMap.arr[currentIndex].source = source;
	correspondenceMap.arr[currentIndex].sink = sink;	
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
	return correspondenceMap.arr[correspondenceIndex].source;
	}

Point2D GetSink(const CorrespondenceMap2D& correspondenceMap, const int correspondenceIndex)
	{
	ASSERT_ON_TEST(correspondenceIndex < correspondenceMap.nCount, "A missing correspondence was requested from a correspondence map 3d");
	return correspondenceMap.arr[correspondenceIndex].sink;
	}

T_Float GetProbability(const CorrespondenceMap2D& correspondenceMap, const int correspondenceIndex)
	{
	ASSERT_ON_TEST(correspondenceIndex < correspondenceMap.nCount, "A missing correspondence was requested from a correspondence map 3d");
	return correspondenceMap.arr[correspondenceIndex].probability;
	}

}

/** @} */
