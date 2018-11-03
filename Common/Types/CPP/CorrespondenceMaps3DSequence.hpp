/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup CorrespondenceMaps3DWrapper
 *
 * Wrapper for ASN.1 Correspondence Maps 3D Sequence types
 *
 * @{
 */

#ifndef CORRESPONDENCE_MAPS_3D_SEQUENCE_HPP
#define CORRESPONDENCE_MAPS_3D_SEQUENCE_HPP

#include <Types/C/Sequences.h>

#include "BaseTypes.hpp"
#include "CorrespondenceMap3D.hpp"
#include <stdlib.h>
#include <memory>
#include <vector>

namespace CorrespondenceMap3DWrapper
{

// Types
typedef asn1SccCorrespondenceMaps3DSequence CorrespondenceMaps3DSequence;

// Global constant variables
const int MAX_CORRESPONDENCE_MAPS_SEQUENCE_LENGTH = maxCorrespondenceMapsSequenceLength;

// Pointer types

typedef CorrespondenceMaps3DSequence* CorrespondenceMaps3DSequencePtr;
typedef CorrespondenceMaps3DSequence const* CorrespondenceMaps3DSequenceConstPtr;
typedef std::shared_ptr<CorrespondenceMaps3DSequence> CorrespondenceMaps3DSequenceSharedPtr;
typedef std::shared_ptr<const CorrespondenceMaps3DSequence> CorrespondenceMaps3DSequenceSharedConstPtr;

// Functions

void Copy(const CorrespondenceMaps3DSequence& source, CorrespondenceMaps3DSequence& destination);
CorrespondenceMaps3DSequencePtr NewCorrespondenceMaps3DSequence();
CorrespondenceMaps3DSequenceSharedPtr NewSharedCorrespondenceMaps3DSequence();
CorrespondenceMaps3DSequenceConstPtr Clone(const CorrespondenceMaps3DSequence& source);
CorrespondenceMaps3DSequenceSharedPtr SharedClone(const CorrespondenceMaps3DSequence& source);
void Initialize(CorrespondenceMaps3DSequence& correspondenceMapsSequence);

void Clear(CorrespondenceMaps3DSequence& correspondenceMapsSequence);
void AddCorrespondenceMap(CorrespondenceMaps3DSequence& correspondenceMapsSequence, const CorrespondenceMap3D& correspondenceMap);
const CorrespondenceMap3D& GetCorrespondenceMap(const CorrespondenceMaps3DSequence& correspondenceMapsSequence, BaseTypesWrapper::T_UInt32 correspondenceMapIndex);
void GetCorrespondenceMap(const CorrespondenceMaps3DSequence& correspondenceMapsSequence, BaseTypesWrapper::T_UInt32 frameIndex, CorrespondenceMap3D& correspondenceMap);
BaseTypesWrapper::T_UInt32 GetNumberOfCorrespondenceMaps(const CorrespondenceMaps3DSequence& correspondenceMapsSequence);
void RemoveCorrespondenceMaps(CorrespondenceMaps3DSequence& correspondenceMapsSequence, std::vector<BaseTypesWrapper::T_UInt32> correspondenceMapIndexOrderedList);
void RemoveCorrespondences(CorrespondenceMaps3DSequence& correspondenceMapsSequence, BaseTypesWrapper::T_UInt32 mapIndex, std::vector<BaseTypesWrapper::T_UInt32> correspondenceIndexOrderedList);

BitStream ConvertToBitStream(const CorrespondenceMaps3DSequence& sequence);
void ConvertFromBitStream(BitStream bitStream, CorrespondenceMaps3DSequence& sequence);
}

#endif // CORRESPONDENCE_MAPS_3D_SEQUENCE_HPP

/** @} */
