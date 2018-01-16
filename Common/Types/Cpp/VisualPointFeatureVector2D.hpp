/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector2D.hpp
 * @date 15/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup VisualPointFeatureVector2DWrapper
 * 
 * VisualPointFeatureVector2D wrapper for the VisualPointFeatureVector2D type
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
#include <VisualPointFeatureVector2D.h>
}
#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>



#ifndef VISUAL_POINT_FEATURE_VECTOR_HPP
#define VISUAL_POINT_FEATURE_VECTOR_HPP

namespace VisualPointFeatureVector2DWrapper 
{
/* --------------------------------------------------------------------------
 *
 * Cpp typedef definition
 *
 * --------------------------------------------------------------------------
 */

typedef CTypes::VisualPointFeature2D_descriptor VisualPointDescriptor2D;
typedef CTypes::VisualPointFeature2D VisualPointFeature2D;
typedef CTypes::VisualPointFeatureVector2D VisualPointFeatureVector2D;


/* --------------------------------------------------------------------------
 *
 * Constants
 *
 * --------------------------------------------------------------------------
 */
const int MAX_FEATURE_2D_POINTS = static_cast<int>(CTypes::features2DElementsMax);
const int MAX_DESCRIPTOR_2D_LENGTH = static_cast<int>(CTypes::descriptor2DNameLength);


/* --------------------------------------------------------------------------
 *
 * Shared Pointers definition
 *
 * --------------------------------------------------------------------------
 */
typedef std::shared_ptr<VisualPointFeatureVector2D> VisualPointFeatureVector2DPtr;
typedef std::shared_ptr<const VisualPointFeatureVector2D> VisualPointFeatureVector2DConstPtr;


/* --------------------------------------------------------------------------
 *
 * Access Functions definition
 *
 * --------------------------------------------------------------------------
 */
void Copy(const VisualPointFeatureVector2D& source, VisualPointFeatureVector2D& destination);

void AddPoint(VisualPointFeatureVector2D& featuresVector, uint16_t x, uint16_t y);
void ClearPoints(VisualPointFeatureVector2D& featuresVector);
int GetNumberOfPoints(const VisualPointFeatureVector2D& featuresVector);
int GetXCoordinate(const VisualPointFeatureVector2D& featuresVector, int pointIndex);
int GetYCoordinate(const VisualPointFeatureVector2D& featuresVector, int pointIndex);

void AddDescriptorComponent(VisualPointFeatureVector2D& featuresVector, int pointIndex, float component);
void ClearDescriptor(VisualPointFeatureVector2D& featuresVector, int pointIndex);
int GetNumberOfDescriptorComponents(const VisualPointFeatureVector2D& featuresVector, int pointIndex);
float GetDescriptorComponent(const VisualPointFeatureVector2D& featuresVector, int pointIndex, int componentIndex);

}
#endif

/* VisualPointFeatureVector2D.hpp */
/** @} */
