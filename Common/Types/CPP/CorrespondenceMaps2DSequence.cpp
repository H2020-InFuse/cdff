/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup CorrespondenceMap2DWrapper
 * @{
 */

#include "CorrespondenceMaps2DSequence.hpp"
#include <Errors/Assert.hpp>

namespace CorrespondenceMap2DWrapper
{

using namespace BaseTypesWrapper;

void Copy(const CorrespondenceMaps2DSequence& source, CorrespondenceMaps2DSequence& destination)
{
	Clear(destination);
	for(T_UInt32 correspondenceMapIndex = 0; correspondenceMapIndex < GetNumberOfCorrespondenceMaps(source); correspondenceMapIndex++)
		{
		AddCorrespondenceMap(destination, GetCorrespondenceMap(source, correspondenceMapIndex));
		}
}

CorrespondenceMaps2DSequencePtr NewCorrespondenceMaps2DSequence()
{
	CorrespondenceMaps2DSequencePtr correspondenceMapsSequence = new CorrespondenceMaps2DSequence();
	Initialize(*correspondenceMapsSequence);
	return correspondenceMapsSequence;
}

CorrespondenceMaps2DSequenceSharedPtr NewSharedCorrespondenceMaps2DSequence()
{
	CorrespondenceMaps2DSequenceSharedPtr sharedCorrespondenceMapsSequence = std::make_shared<CorrespondenceMaps2DSequence>();
	Initialize(*sharedCorrespondenceMapsSequence);
	return sharedCorrespondenceMapsSequence;
}

CorrespondenceMaps2DSequenceConstPtr Clone(const CorrespondenceMaps2DSequence& source)
{
	CorrespondenceMaps2DSequencePtr correspondenceMapsSequence = new CorrespondenceMaps2DSequence();
	Copy(source, *correspondenceMapsSequence);
	return correspondenceMapsSequence;
}

CorrespondenceMaps2DSequenceSharedPtr SharedClone(const CorrespondenceMaps2DSequence& source)
{
	CorrespondenceMaps2DSequenceSharedPtr sharedCorrespondenceMapsSequence = std::make_shared<CorrespondenceMaps2DSequence>();
	Copy(source, *sharedCorrespondenceMapsSequence);
	return sharedCorrespondenceMapsSequence;
}

void Initialize(CorrespondenceMaps2DSequence& correspondenceMapsSequence)
{
	Clear(correspondenceMapsSequence);
}

void Clear(CorrespondenceMaps2DSequence& correspondenceMapsSequence)
{
	correspondenceMapsSequence.nCount = 0;
}


void AddCorrespondenceMap(CorrespondenceMaps2DSequence& correspondenceMapsSequence, const CorrespondenceMap2D& correspondenceMap)
{
	ASSERT( GetNumberOfCorrespondenceMaps(correspondenceMapsSequence) < MAX_CORRESPONDENCE_MAPS_SEQUENCE_LENGTH, "Error, correspondenceMaps sequence limit reached");
	Copy( correspondenceMap, correspondenceMapsSequence.arr[correspondenceMapsSequence.nCount] );
	correspondenceMapsSequence.nCount++;
}

const CorrespondenceMap2D& GetCorrespondenceMap(const CorrespondenceMaps2DSequence& correspondenceMapsSequence, BaseTypesWrapper::T_UInt32 correspondenceMapIndex)
{
	ASSERT(correspondenceMapIndex >= 0 && correspondenceMapIndex < correspondenceMapsSequence.nCount, "GetCorrespondenceMap error, correspondenceMapIndex out of range");
	return correspondenceMapsSequence.arr[correspondenceMapIndex];
}

void GetCorrespondenceMap(const CorrespondenceMaps2DSequence& correspondenceMapsSequence, BaseTypesWrapper::T_UInt32 correspondenceMapIndex, CorrespondenceMap2D& correspondenceMap)
{
	Copy( GetCorrespondenceMap(correspondenceMapsSequence, correspondenceMapIndex), correspondenceMap);
}

BaseTypesWrapper::T_UInt32 GetNumberOfCorrespondenceMaps(const CorrespondenceMaps2DSequence& correspondenceMapsSequence)
{
	return correspondenceMapsSequence.nCount;
}

void RemoveCorrespondenceMaps(CorrespondenceMaps2DSequence& correspondenceMapsSequence, std::vector<BaseTypesWrapper::T_UInt32> correspondenceMapIndexOrderedList)
	{
	BaseTypesWrapper::T_UInt32 elementsToRemove = correspondenceMapIndexOrderedList.size();
	if ( elementsToRemove == 0)
		{
		return;
		}

	//Checking that input is ordered correctly.
	static const std::string errorMessage = "Remove Correspondence Map error, the second input was not an ORDERED list or some index is not within range";
	for(int listIndex = 1; listIndex < elementsToRemove-1; listIndex++)
		{
		ASSERT( correspondenceMapIndexOrderedList.at(listIndex-1) < correspondenceMapIndexOrderedList.at(listIndex), errorMessage);
		ASSERT(	correspondenceMapIndexOrderedList.at(listIndex) < correspondenceMapIndexOrderedList.at(listIndex+1), errorMessage);
		}
	ASSERT( correspondenceMapIndexOrderedList.at(elementsToRemove-1) < correspondenceMapsSequence.nCount, errorMessage);
	BaseTypesWrapper::T_UInt32 firstIndex = correspondenceMapIndexOrderedList.at(0);
	ASSERT(firstIndex >= 0, errorMessage); 

	//Removing elements
	BaseTypesWrapper::T_UInt32 nextIndexToRemove = 1;
	BaseTypesWrapper::T_UInt32 currentGap = 1;
	for(int mapIndex = firstIndex; mapIndex < correspondenceMapsSequence.nCount - elementsToRemove; mapIndex++)
		{
		if (nextIndexToRemove < elementsToRemove && mapIndex+currentGap == correspondenceMapIndexOrderedList.at(nextIndexToRemove))
			{
			currentGap++;
			nextIndexToRemove++;
			mapIndex--; //This is to not allow the map index to step forward in the next iteration;
			}
		else
			{
			Copy(correspondenceMapsSequence.arr[mapIndex+currentGap], correspondenceMapsSequence.arr[mapIndex]);
			}
		}
	correspondenceMapsSequence.nCount -= elementsToRemove;
	}

void RemoveCorrespondences(CorrespondenceMaps2DSequence& correspondenceMapsSequence, BaseTypesWrapper::T_UInt32 mapIndex, std::vector<BaseTypesWrapper::T_UInt32> correspondenceIndexOrderedList)
	{
	ASSERT(mapIndex >= 0 && mapIndex < correspondenceMapsSequence.nCount, "RemoveCorrespondences error, mapIndex out of range");
	RemoveCorrespondences(correspondenceMapsSequence.arr[mapIndex], correspondenceIndexOrderedList);
	}

BitStream ConvertToBitStream(const CorrespondenceMaps2DSequence& sequence)
	CONVERT_TO_BIT_STREAM(sequence, asn1SccCorrespondenceMaps2DSequence_REQUIRED_BYTES_FOR_ENCODING, asn1SccCorrespondenceMaps2DSequence_Encode)

void ConvertFromBitStream(BitStream bitStream, CorrespondenceMaps2DSequence& sequence)
	CONVERT_FROM_BIT_STREAM(bitStream, asn1SccCorrespondenceMaps2DSequence_REQUIRED_BYTES_FOR_ENCODING, sequence, asn1SccCorrespondenceMaps2DSequence_Decode)

}

/** @} */
