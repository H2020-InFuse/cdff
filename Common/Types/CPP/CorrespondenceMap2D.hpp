/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup CorrespondenceMap2DWrapper
 *
 * Wrapper for the ASN.1 CorrespondenceMap2D type
 *
 * @{
 */

#ifndef CORRESPONDENCE_MAP_2D_HPP
#define CORRESPONDENCE_MAP_2D_HPP

#include <CorrespondenceMap2D.h>

#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>
#include <vector>

namespace CorrespondenceMap2DWrapper
{

// Types

typedef asn1SccCorrespondence2D Correspondence2D;
typedef asn1SccCorrespondenceMap2D CorrespondenceMap2D;

// Global constant variables

const int MAX_CORRESPONDENCES_2D = static_cast<int>(correspondenceMap2DElementsMax);

// Pointer types

typedef CorrespondenceMap2D* CorrespondenceMap2DPtr;
typedef CorrespondenceMap2D const* CorrespondenceMap2DConstPtr;
typedef std::shared_ptr<CorrespondenceMap2D> CorrespondenceMap2DSharedPtr;
typedef std::shared_ptr<const CorrespondenceMap2D> CorrespondenceMap2DSharedConstPtr;

// Functions

void Copy(const CorrespondenceMap2D& source, CorrespondenceMap2D& destination);
CorrespondenceMap2DPtr NewCorrespondenceMap2D();
CorrespondenceMap2DSharedPtr NewSharedCorrespondenceMap2D();
void Initialize(CorrespondenceMap2D& correspondenceMap);

void AddCorrespondence(CorrespondenceMap2D& correspondenceMap, BaseTypesWrapper::Point2D source, BaseTypesWrapper::Point2D sink, BaseTypesWrapper::T_Float probability);
void ClearCorrespondences(CorrespondenceMap2D& correspondenceMap);
int GetNumberOfCorrespondences(const CorrespondenceMap2D& correspondenceMap);
BaseTypesWrapper::Point2D GetSource(const CorrespondenceMap2D& correspondenceMap, const int correspondenceIndex);
BaseTypesWrapper::Point2D GetSink(const CorrespondenceMap2D& correspondenceMap, const int correspondenceIndex);
BaseTypesWrapper::T_Float GetProbability(const CorrespondenceMap2D& correspondenceMap, const int correspondenceIndex);
void RemoveCorrespondences(CorrespondenceMap2D& correspondenceMap, std::vector<BaseTypesWrapper::T_UInt32> correspondenceIndexOrderedList);

BitStream ConvertToBitStream(const CorrespondenceMap2D& map);
void ConvertFromBitStream(BitStream bitStream, CorrespondenceMap2D& map);

}

#endif // CORRESPONDENCE_MAP_2D_HPP

/** @} */
