/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CorrespondenceMap3D.hpp
 * @date 17/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CorrespondenceMap3DWrapper
 * 
 * CorrespondenceMap3D wrapper for the CorrespondenceMap3D type
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
namespace CTypes {
#include <CorrespondenceMap3D.h>
}
#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>



#ifndef CORRESPONDENCE_MAP_3D_HPP
#define CORRESPONDENCE_MAP_3D_HPP

namespace CorrespondenceMap3DWrapper 
{
/* --------------------------------------------------------------------------
 *
 * Cpp typedef definition
 *
 * --------------------------------------------------------------------------
 */

typedef CTypes::Correspondence3D Correspondence3D;
typedef CTypes::CorrespondenceMap3D CorrespondenceMap3D;


/* --------------------------------------------------------------------------
 *
 * Constants
 *
 * --------------------------------------------------------------------------
 */
const int MAX_CORRESPONDENCES_3D = static_cast<int>(CTypes::correspondeceMap3DElementsMax);



/* --------------------------------------------------------------------------
 *
 * Shared Pointers definition
 *
 * --------------------------------------------------------------------------
 */
typedef std::shared_ptr<CorrespondenceMap3D> CorrespondenceMap3DSharedPtr;
typedef std::shared_ptr<const CorrespondenceMap3D> CorrespondenceMap3DSharedConstPtr;
typedef CorrespondenceMap3D* CorrespondenceMap3DPtr;
typedef CorrespondenceMap3D const* CorrespondenceMap3DConstPtr;

/* --------------------------------------------------------------------------
 *
 * Access Functions definition
 *
 * --------------------------------------------------------------------------
 */
void Copy(const CorrespondenceMap3D& source, CorrespondenceMap3D& destination);

void AddCorrespondence(CorrespondenceMap3D& correspondenceMap, BaseTypesWrapper::Point3D source, BaseTypesWrapper::Point3D sink, BaseTypesWrapper::T_Float probability);
void ClearCorrespondences(CorrespondenceMap3D& correspondenceMap);
int GetNumberOfCorrespondences(const CorrespondenceMap3D& correspondenceMap);
BaseTypesWrapper::Point3D GetSource(const CorrespondenceMap3D& correspondenceMap, const int correspondenceIndex);
BaseTypesWrapper::Point3D GetSink(const CorrespondenceMap3D& correspondenceMap, const int correspondenceIndex);
BaseTypesWrapper::T_Float GetProbability(const CorrespondenceMap3D& correspondenceMap, const int correspondenceIndex);

}
#endif

/* CorrespondenceMap3D.hpp */
/** @} */
