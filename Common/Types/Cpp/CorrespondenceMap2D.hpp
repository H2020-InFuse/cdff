/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CorrespondenceMap2D.hpp
 * @date 26/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CorrespondenceMap2DWrapper
 * 
 * CorrespondenceMap2D wrapper for the CorrespondenceMap2D type
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
#include <CorrespondenceMap2D.h>
}
#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>



#ifndef CORRESPONDENCE_MAP_2D_HPP
#define CORRESPONDENCE_MAP_2D_HPP

namespace CorrespondenceMap2DWrapper 
{
/* --------------------------------------------------------------------------
 *
 * Cpp typedef definition
 *
 * --------------------------------------------------------------------------
 */

typedef CTypes::Correspondence2D Correspondence2D;
typedef CTypes::CorrespondenceMap2D CorrespondenceMap2D;


/* --------------------------------------------------------------------------
 *
 * Constants
 *
 * --------------------------------------------------------------------------
 */
const int MAX_CORRESPONDENCES_2D = static_cast<int>(CTypes::correspondenceMap2DElementsMax);



/* --------------------------------------------------------------------------
 *
 * Shared Pointers definition
 *
 * --------------------------------------------------------------------------
 */
typedef std::shared_ptr<CorrespondenceMap2D> CorrespondenceMap2DSharedPtr;
typedef std::shared_ptr<const CorrespondenceMap2D> CorrespondenceMap2DSharedConstPtr;
typedef CorrespondenceMap2D* CorrespondenceMap2DPtr;
typedef CorrespondenceMap2D const* CorrespondenceMap2DConstPtr;

/* --------------------------------------------------------------------------
 *
 * Access Functions definition
 *
 * --------------------------------------------------------------------------
 */
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

}
#endif

/* CorrespondenceMap2D.hpp */
/** @} */
