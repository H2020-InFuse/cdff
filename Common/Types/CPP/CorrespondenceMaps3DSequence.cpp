/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup CorrespondenceMap3DWrapper
 * @{
 */

#include "CorrespondenceMaps3DSequence.hpp"
#include "Errors/AssertOnTest.hpp"

namespace CorrespondenceMap3DWrapper
{

using namespace BaseTypesWrapper;

void Copy(const CorrespondenceMaps3DSequence& source, CorrespondenceMaps3DSequence& destination)
{
	Clear(destination);
	T_UInt32 numberOfMaps = GetNumberOfCorrespondenceMaps(source);
	for(T_UInt32 correspondenceMapIndex = 0; correspondenceMapIndex < numberOfMaps; correspondenceMapIndex++)
		{
		AddCorrespondenceMap(destination, GetCorrespondenceMap(source, correspondenceMapIndex));
		}
}

CorrespondenceMaps3DSequencePtr NewCorrespondenceMaps3DSequence()
{
	CorrespondenceMaps3DSequencePtr correspondenceMapsSequence = new CorrespondenceMaps3DSequence();
	Initialize(*correspondenceMapsSequence);
	return correspondenceMapsSequence;
}

CorrespondenceMaps3DSequenceSharedPtr NewSharedCorrespondenceMaps3DSequence()
{
	CorrespondenceMaps3DSequenceSharedPtr sharedCorrespondenceMapsSequence = std::make_shared<CorrespondenceMaps3DSequence>();
	Initialize(*sharedCorrespondenceMapsSequence);
	return sharedCorrespondenceMapsSequence;
}

CorrespondenceMaps3DSequenceConstPtr Clone(const CorrespondenceMaps3DSequence& source)
{
	CorrespondenceMaps3DSequencePtr correspondenceMapsSequence = new CorrespondenceMaps3DSequence();
	Copy(source, *correspondenceMapsSequence);
	return correspondenceMapsSequence;
}

CorrespondenceMaps3DSequenceSharedPtr SharedClone(const CorrespondenceMaps3DSequence& source)
{
	CorrespondenceMaps3DSequenceSharedPtr sharedCorrespondenceMapsSequence = std::make_shared<CorrespondenceMaps3DSequence>();
	Copy(source, *sharedCorrespondenceMapsSequence);
	return sharedCorrespondenceMapsSequence;
}

void Initialize(CorrespondenceMaps3DSequence& correspondenceMapsSequence)
{
	Clear(correspondenceMapsSequence);
}

void Clear(CorrespondenceMaps3DSequence& correspondenceMapsSequence)
{
	correspondenceMapsSequence.nCount = 0;
}


void AddCorrespondenceMap(CorrespondenceMaps3DSequence& correspondenceMapsSequence, const CorrespondenceMap3D& correspondenceMap)
{
	ASSERT( GetNumberOfCorrespondenceMaps(correspondenceMapsSequence) < MAX_CORRESPONDENCE_MAPS_SEQUENCE_LENGTH, "Error, correspondenceMaps sequence limit reached");
	Copy( correspondenceMap, correspondenceMapsSequence.arr[correspondenceMapsSequence.nCount] );
	correspondenceMapsSequence.nCount++;
}

const CorrespondenceMap3D& GetCorrespondenceMap(const CorrespondenceMaps3DSequence& correspondenceMapsSequence, BaseTypesWrapper::T_UInt32 correspondenceMapIndex)
{
	ASSERT(correspondenceMapIndex < correspondenceMapsSequence.nCount, "GetCorrespondenceMap error, correspondenceMapIndex out of range");
	return correspondenceMapsSequence.arr[correspondenceMapIndex];
}

void GetCorrespondenceMap(const CorrespondenceMaps3DSequence& correspondenceMapsSequence, BaseTypesWrapper::T_UInt32 correspondenceMapIndex, CorrespondenceMap3D& correspondenceMap)
{
	Copy( GetCorrespondenceMap(correspondenceMapsSequence, correspondenceMapIndex), correspondenceMap);
}

BaseTypesWrapper::T_UInt32 GetNumberOfCorrespondenceMaps(const CorrespondenceMaps3DSequence& correspondenceMapsSequence)
{
	return correspondenceMapsSequence.nCount;
}

void RemoveCorrespondenceMaps(CorrespondenceMaps3DSequence& correspondenceMapsSequence, std::vector<BaseTypesWrapper::T_UInt32> correspondenceMapIndexOrderedList)
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

void RemoveCorrespondences(CorrespondenceMaps3DSequence& correspondenceMapsSequence, BaseTypesWrapper::T_UInt32 mapIndex, const std::vector<BaseTypesWrapper::T_UInt32>& correspondenceIndexOrderedList)
	{
	ASSERT(mapIndex < correspondenceMapsSequence.nCount, "RemoveCorrespondences error, mapIndex out of range");
	RemoveCorrespondences(correspondenceMapsSequence.arr[mapIndex], correspondenceIndexOrderedList);
	}

BitStream ConvertToBitStream(const CorrespondenceMaps3DSequence& sequence)
	{
	return BaseTypesWrapper::ConvertToBitStream(sequence, asn1SccCorrespondenceMaps3DSequence_REQUIRED_BYTES_FOR_ENCODING, asn1SccCorrespondenceMaps3DSequence_Encode);
	}
void ConvertFromBitStream(BitStream bitStream, CorrespondenceMaps3DSequence& sequence)
	{
	BaseTypesWrapper::ConvertFromBitStream(bitStream, asn1SccCorrespondenceMaps3DSequence_REQUIRED_BYTES_FOR_ENCODING, sequence, asn1SccCorrespondenceMaps3DSequence_Decode);
	}

}

/** @} */
