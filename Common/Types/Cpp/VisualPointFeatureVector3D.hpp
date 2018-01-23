/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector3D.hpp
 * @date 15/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup VisualPointFeatureVector3DWrapper
 * 
 * VisualPointFeatureVector3D wrapper for the VisualPointFeatureVector3D type
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
#include <VisualPointFeatureVector3D.h>
}
#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>



#ifndef VISUAL_POINT_FEATURE_VECTOR_3D_HPP
#define VISUAL_POINT_FEATURE_VECTOR_3D_HPP

namespace VisualPointFeatureVector3DWrapper 
{
/* --------------------------------------------------------------------------
 *
 * Cpp typedef definition
 *
 * --------------------------------------------------------------------------
 */

typedef CTypes::VisualPointFeature3D_descriptor VisualPointDescriptor3D;
typedef CTypes::VisualPointFeature3D VisualPointFeature3D;
typedef CTypes::VisualPointFeatureVector3D VisualPointFeatureVector3D;


/* --------------------------------------------------------------------------
 *
 * Constants
 *
 * --------------------------------------------------------------------------
 */
const int MAX_FEATURE_3D_POINTS = static_cast<int>(CTypes::features3DElementsMax);
const int MAX_DESCRIPTOR_3D_LENGTH = static_cast<int>(CTypes::descriptor3DNameLength);



/* --------------------------------------------------------------------------
 *
 * Shared Pointers definition
 *
 * --------------------------------------------------------------------------
 */
typedef std::shared_ptr<VisualPointFeatureVector3D> VisualPointFeatureVector3DSharedPtr;
typedef std::shared_ptr<const VisualPointFeatureVector3D> VisualPointFeatureVector3DSharedConstPtr;
typedef VisualPointFeatureVector3D* VisualPointFeatureVector3DPtr;
typedef VisualPointFeatureVector3D const* VisualPointFeatureVector3DConstPtr;

/* --------------------------------------------------------------------------
 *
 * Access Functions definition
 *
 * --------------------------------------------------------------------------
 */
void Copy(const VisualPointFeatureVector3D& source, VisualPointFeatureVector3D& destination);

void AddPoint(VisualPointFeatureVector3D& featuresVector, float x, float y, float z);
void ClearPoints(VisualPointFeatureVector3D& featuresVector);
int GetNumberOfPoints(const VisualPointFeatureVector3D& featuresVector);
float GetXCoordinate(const VisualPointFeatureVector3D& featuresVector, int pointIndex);
float GetYCoordinate(const VisualPointFeatureVector3D& featuresVector, int pointIndex);
float GetZCoordinate(const VisualPointFeatureVector3D& featuresVector, int pointIndex);

void AddDescriptorComponent(VisualPointFeatureVector3D& featuresVector, int pointIndex, float component);
void ClearDescriptor(VisualPointFeatureVector3D& featuresVector, int pointIndex);
int GetNumberOfDescriptorComponents(const VisualPointFeatureVector3D& featuresVector, int pointIndex);
float GetDescriptorComponent(const VisualPointFeatureVector3D& featuresVector, int pointIndex, int componentIndex);

}
#endif

/* VisualPointFeatureVector3D.hpp */
/** @} */
