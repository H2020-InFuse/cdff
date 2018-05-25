/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup CorrespondenceMap3DWrapper
 *
 * Wrapper for the ASN.1 CorrespondenceMap3D type
 *
 * @{
 */

#ifndef CORRESPONDENCE_MAP_3D_HPP
#define CORRESPONDENCE_MAP_3D_HPP

#include <CorrespondenceMap3D.h>

#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>

namespace CorrespondenceMap3DWrapper
{

// Types

typedef asn1SccCorrespondence3D Correspondence3D;
typedef asn1SccCorrespondenceMap3D CorrespondenceMap3D;

// Global constant variables

const int MAX_CORRESPONDENCES_3D = static_cast<int>(correspondeceMap3DElementsMax);

// Pointer types

typedef CorrespondenceMap3D* CorrespondenceMap3DPtr;
typedef CorrespondenceMap3D const* CorrespondenceMap3DConstPtr;
typedef std::shared_ptr<CorrespondenceMap3D> CorrespondenceMap3DSharedPtr;
typedef std::shared_ptr<const CorrespondenceMap3D> CorrespondenceMap3DSharedConstPtr;

// Functions

void Copy(const CorrespondenceMap3D& source, CorrespondenceMap3D& destination);
CorrespondenceMap3DPtr NewCorrespondenceMap3D();
CorrespondenceMap3DSharedPtr NewSharedCorrespondenceMap3D();
void Initialize(CorrespondenceMap3D& correspondenceMap);

void AddCorrespondence(CorrespondenceMap3D& correspondenceMap, BaseTypesWrapper::Point3D source, BaseTypesWrapper::Point3D sink, BaseTypesWrapper::T_Float probability);
void ClearCorrespondences(CorrespondenceMap3D& correspondenceMap);
int GetNumberOfCorrespondences(const CorrespondenceMap3D& correspondenceMap);
BaseTypesWrapper::Point3D GetSource(const CorrespondenceMap3D& correspondenceMap, const int correspondenceIndex);
BaseTypesWrapper::Point3D GetSink(const CorrespondenceMap3D& correspondenceMap, const int correspondenceIndex);
BaseTypesWrapper::T_Float GetProbability(const CorrespondenceMap3D& correspondenceMap, const int correspondenceIndex);

}

#endif // CORRESPONDENCE_MAP_3D_HPP

/** @} */
