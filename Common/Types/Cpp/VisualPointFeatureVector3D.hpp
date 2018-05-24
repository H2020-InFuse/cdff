/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup VisualPointFeatureVector3DWrapper
 *
 * Wrapper for the ASN.1 VisualPointFeatureVector3D type
 *
 * @{
 */

#ifndef VISUAL_POINT_FEATURE_VECTOR_3D_HPP
#define VISUAL_POINT_FEATURE_VECTOR_3D_HPP

#include <VisualPointFeatureVector3D.h>

#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>

namespace VisualPointFeatureVector3DWrapper 
{

// Types

typedef asn1SccVisualPointFeature3D_descriptor VisualPointDescriptor3D;
typedef asn1SccVisualPointFeature3D VisualPointFeature3D;
typedef asn1SccVisualPointFeatureVector3D VisualPointFeatureVector3D;
typedef asn1SccVisualPointFeature3D_point VisualPoint;
typedef asn1SccVisualPointFeatureVector3DType VisualPointFeatureVector3DType;
typedef asn1SccVisualPointFeatureVector3D_list VisualPointFeatureVector3DList;

// Enumerated types

enum VisualPointType // give a name to an anonymous enum in VisualPoint
{
	VISUAL_POINT_NONE,
	VISUAL_POINT_POSITION,
	VISUAL_POINT_REFERENCE
};

const VisualPointFeatureVector3DType ALL_POSITIONS_VECTOR = asn1Sccall_positions;
const VisualPointFeatureVector3DType ALL_REFERENCES_VECTOR = asn1Sccall_references;
const VisualPointFeatureVector3DType HYBRID_VECTOR = asn1Scchybrid_vector;

// Global constant variables

const int MAX_FEATURE_3D_POINTS = static_cast<int>(features3DElementsMax);
const int MAX_DESCRIPTOR_3D_LENGTH = static_cast<int>(descriptor3DNameLength);

// Pointer types

typedef VisualPointFeatureVector3D* VisualPointFeatureVector3DPtr;
typedef VisualPointFeatureVector3D const* VisualPointFeatureVector3DConstPtr;
typedef std::shared_ptr<VisualPointFeatureVector3D> VisualPointFeatureVector3DSharedPtr;
typedef std::shared_ptr<const VisualPointFeatureVector3D> VisualPointFeatureVector3DSharedConstPtr;

// Functions

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

#endif // VISUAL_POINT_FEATURE_VECTOR_3D_HPP

/** @} */
