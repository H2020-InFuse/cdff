/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CorrespondenceMap3D.cpp
 * @date 17/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CorrespondenceMap3DWrapper
 * 
 * Implementation of CorrespondenceMap3DWrapper functions.
 * 
 * 
 * @{
 */


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "CorrespondenceMap3D.hpp"
#include <Errors/Assert.hpp>

namespace CorrespondenceMap3DWrapper
{

using namespace BaseTypesWrapper;

/* --------------------------------------------------------------------------
 *
 * Functions
 *
 * --------------------------------------------------------------------------
 */
void Copy(const CorrespondenceMap3D& source, CorrespondenceMap3D& destination)
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
	correspondenceMap.arr[currentIndex].source = source;
	correspondenceMap.arr[currentIndex].sink = sink;	
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
	return correspondenceMap.arr[correspondenceIndex].source;
	}

Point3D GetSink(const CorrespondenceMap3D& correspondenceMap, const int correspondenceIndex)
	{
	ASSERT_ON_TEST(correspondenceIndex < correspondenceMap.nCount, "A missing correspondence was requested from a correspondence map 3d");
	return correspondenceMap.arr[correspondenceIndex].sink;
	}

T_Float GetProbability(const CorrespondenceMap3D& correspondenceMap, const int correspondenceIndex)
	{
	ASSERT_ON_TEST(correspondenceIndex < correspondenceMap.nCount, "A missing correspondence was requested from a correspondence map 3d");
	return correspondenceMap.arr[correspondenceIndex].probability;
	}

}

/** @} */
