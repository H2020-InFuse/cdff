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
	for(T_UInt32 correspondenceMapIndex = 0; correspondenceMapIndex < GetNumberOfCorrespondenceMaps(destination); correspondenceMapIndex++)
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

}

/** @} */
