/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CorrespondenceMap2D.cpp
 * @date 26/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CorrespondenceMap2DWrapper
 * 
 * Implementation of CorrespondenceMap2DWrapper functions.
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
#include "CorrespondenceMap2D.hpp"
#include <Errors/Assert.hpp>

namespace CorrespondenceMap2DWrapper
{

using namespace BaseTypesWrapper;

/* --------------------------------------------------------------------------
 *
 * Functions
 *
 * --------------------------------------------------------------------------
 */
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
