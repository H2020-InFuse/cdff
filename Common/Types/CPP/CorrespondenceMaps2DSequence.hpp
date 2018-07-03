/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup CorrespondenceMaps2DWrapper
 *
 * Wrapper for ASN.1 Correspondence Maps 2D Sequence types
 *
 * @{
 */

#ifndef CORRESPONDENCE_MAPS_2D_SEQUENCE_HPP
#define CORRESPONDENCE_MAPS_2D_SEQUENCE_HPP

#include <Sequences.h>

#include "BaseTypes.hpp"
#include <CorrespondenceMap2D.hpp>
#include <stdlib.h>
#include <memory>
#include <vector>

namespace CorrespondenceMap2DWrapper
{

// Types
typedef asn1SccCorrespondenceMaps2DSequence CorrespondenceMaps2DSequence;

// Global constant variables
const int MAX_CORRESPONDENCE_MAPS_SEQUENCE_LENGTH = maxCorrespondenceMapsSequenceLength;

// Pointer types

typedef CorrespondenceMaps2DSequence* CorrespondenceMaps2DSequencePtr;
typedef CorrespondenceMaps2DSequence const* CorrespondenceMaps2DSequenceConstPtr;
typedef std::shared_ptr<CorrespondenceMaps2DSequence> CorrespondenceMaps2DSequenceSharedPtr;
typedef std::shared_ptr<const CorrespondenceMaps2DSequence> CorrespondenceMaps2DSequenceSharedConstPtr;

// Functions

void Copy(const CorrespondenceMaps2DSequence& source, CorrespondenceMaps2DSequence& destination);
CorrespondenceMaps2DSequencePtr NewCorrespondenceMaps2DSequence();
CorrespondenceMaps2DSequenceSharedPtr NewSharedCorrespondenceMaps2DSequence();
CorrespondenceMaps2DSequenceConstPtr Clone(const CorrespondenceMaps2DSequence& source);
CorrespondenceMaps2DSequenceSharedPtr SharedClone(const CorrespondenceMaps2DSequence& source);
void Initialize(CorrespondenceMaps2DSequence& correspondenceMapsSequence);

void Clear(CorrespondenceMaps2DSequence& correspondenceMapsSequence);
void AddCorrespondenceMap(CorrespondenceMaps2DSequence& correspondenceMapsSequence, const CorrespondenceMap2D& correspondenceMap);
const CorrespondenceMap2D& GetCorrespondenceMap(const CorrespondenceMaps2DSequence& correspondenceMapsSequence, BaseTypesWrapper::T_UInt32 correspondenceMapIndex);
void GetCorrespondenceMap(const CorrespondenceMaps2DSequence& correspondenceMapsSequence, BaseTypesWrapper::T_UInt32 frameIndex, CorrespondenceMap2D& correspondenceMap);
BaseTypesWrapper::T_UInt32 GetNumberOfCorrespondenceMaps(const CorrespondenceMaps2DSequence& correspondenceMapsSequence); 
void RemoveCorrespondenceMaps(CorrespondenceMaps2DSequence& correspondenceMapsSequence, std::vector<BaseTypesWrapper::T_UInt32> correspondenceMapIndexOrderedList);
void RemoveCorrespondences(CorrespondenceMaps2DSequence& correspondenceMapsSequence, BaseTypesWrapper::T_UInt32 mapIndex, std::vector<BaseTypesWrapper::T_UInt32> correspondenceIndexOrderedList);
}

#endif // CORRESPONDENCE_MAPS_2D_SEQUENCE_HPP

/** @} */
