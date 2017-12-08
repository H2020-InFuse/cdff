/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file BaseTypes.hpp
 * @date 08/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CppTypes
 * 
 * C++ typedefs for the ASN.1 base types
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

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
namespace CTypes {
#include <Frame.h>
#include <VisualPointFeatureVector2D.h>
#include <VisualPointFeatureVector3D.h>
#include <Pointcloud.h>
}

#ifndef BASE_TYPES_HPP
#define BASE_TYPES_HPP

namespace CppTypes 
{


/* --------------------------------------------------------------------------
 *
 * Types definition
 *
 * --------------------------------------------------------------------------
 */
typedef CTypes::T_Int64 T_Int64;
typedef CTypes::T_UInt32 T_UInt32;
typedef CTypes::T_UInt16 T_UInt16;
typedef CTypes::T_String T_String;
typedef CTypes::byte byte;
typedef CTypes::T_Double T_Double;

}

#endif


/* BaseTypes.hpp */
/** @} */
