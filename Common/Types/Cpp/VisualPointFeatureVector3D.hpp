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
typedef CTypes::VisualPointFeature3D_point VisualPoint;
typedef CTypes::VisualPointFeatureVector3DType VisualPointFeatureVector3DType;
typedef CTypes::VisualPointFeatureVector3D_list VisualPointFeatureVector3DList;



/* --------------------------------------------------------------------------
 *
 * Giving a name to anonymous Enum in VisualPoint
 *
 * --------------------------------------------------------------------------
 */
enum VisualPointType
	{
	VISUAL_POINT_NONE,
	VISUAL_POINT_POSITION,
	VISUAL_POINT_REFERENCE
	};


/* --------------------------------------------------------------------------
 *
 * Constants
 *
 * --------------------------------------------------------------------------
 */
const int MAX_FEATURE_3D_POINTS = static_cast<int>(CTypes::features3DElementsMax);
const int MAX_DESCRIPTOR_3D_LENGTH = static_cast<int>(CTypes::descriptor3DNameLength);
const VisualPointFeatureVector3DType ALL_POSITIONS_VECTOR = CTypes::all_positions;
const VisualPointFeatureVector3DType ALL_REFERENCES_VECTOR = CTypes::all_references;
const VisualPointFeatureVector3DType HYBRID_VECTOR = CTypes::hybrid_vector;



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
VisualPointFeatureVector3DPtr NewVisualPointFeatureVector3D();
VisualPointFeatureVector3DSharedPtr NewSharedVisualPointFeatureVector3D();
void Initialize(VisualPointFeatureVector3D& featuresVector);

void AddPoint(VisualPointFeatureVector3D& featuresVector, float x, float y, float z);
void AddPoint(VisualPointFeatureVector3D& featuresVector, BaseTypesWrapper::T_UInt64 index, BaseTypesWrapper::T_UInt16 pointCloudIdentifier = 0);
void ClearPoints(VisualPointFeatureVector3D& featuresVector);
VisualPointFeatureVector3DType GetVectorType(const VisualPointFeatureVector3D& featuresVector);
int GetNumberOfPoints(const VisualPointFeatureVector3D& featuresVector);
float GetXCoordinate(const VisualPointFeatureVector3D& featuresVector, int pointIndex);
float GetYCoordinate(const VisualPointFeatureVector3D& featuresVector, int pointIndex);
float GetZCoordinate(const VisualPointFeatureVector3D& featuresVector, int pointIndex);
BaseTypesWrapper::T_UInt64 GetReferenceIndex(const VisualPointFeatureVector3D& featuresVector, int pointIndex);
BaseTypesWrapper::T_UInt16 GetPointCloudIdentifier(const VisualPointFeatureVector3D& featuresVector, int pointIndex);
VisualPointType GetPointType(const VisualPointFeatureVector3D& featuresVector, int pointIndex);
	

void AddDescriptorComponent(VisualPointFeatureVector3D& featuresVector, int pointIndex, float component);
void ClearDescriptor(VisualPointFeatureVector3D& featuresVector, int pointIndex);
int GetNumberOfDescriptorComponents(const VisualPointFeatureVector3D& featuresVector, int pointIndex);
float GetDescriptorComponent(const VisualPointFeatureVector3D& featuresVector, int pointIndex, int componentIndex);

}
#endif

/* VisualPointFeatureVector3D.hpp */
/** @} */
