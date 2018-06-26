/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup VisualPointFeatureVector2DWrapper
 *
 * Wrapper for the ANS.1 VisualPointFeatureVector2D type
 *
 * @{
 */

#ifndef VISUAL_POINT_FEATURE_VECTOR_2D_HPP
#define VISUAL_POINT_FEATURE_VECTOR_2D_HPP

#include <VisualPointFeatureVector2D.h>

#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>

namespace VisualPointFeatureVector2DWrapper
{

// Types

typedef asn1SccVisualPointFeature2D_descriptor VisualPointDescriptor2D;
typedef asn1SccVisualPointFeature2D VisualPointFeature2D;
typedef asn1SccVisualPointFeatureVector2D VisualPointFeatureVector2D;

// Global constant variables

const int MAX_FEATURE_2D_POINTS = static_cast<int>(features2DElementsMax);
const int MAX_DESCRIPTOR_2D_LENGTH = static_cast<int>(descriptor2DNameLength);

// Pointer types

typedef VisualPointFeatureVector2D* VisualPointFeatureVector2DPtr;
typedef VisualPointFeatureVector2D const* VisualPointFeatureVector2DConstPtr;
typedef std::shared_ptr<VisualPointFeatureVector2D> VisualPointFeatureVector2DSharedPtr;
typedef std::shared_ptr<const VisualPointFeatureVector2D> VisualPointFeatureVector2DSharedConstPtr;

// Functions

void Copy(const VisualPointFeatureVector2D& source, VisualPointFeatureVector2D& destination);
VisualPointFeatureVector2DPtr NewVisualPointFeatureVector2D();
VisualPointFeatureVector2DSharedPtr NewSharedVisualPointFeatureVector2D();
void Initialize(VisualPointFeatureVector2D& featuresVector);

void AddPoint(VisualPointFeatureVector2D& featuresVector, uint16_t x, uint16_t y);
void ClearPoints(VisualPointFeatureVector2D& featuresVector);
int GetNumberOfPoints(const VisualPointFeatureVector2D& featuresVector);
int GetXCoordinate(const VisualPointFeatureVector2D& featuresVector, int pointIndex);
int GetYCoordinate(const VisualPointFeatureVector2D& featuresVector, int pointIndex);

void AddDescriptorComponent(VisualPointFeatureVector2D& featuresVector, int pointIndex, float component);
void ClearDescriptor(VisualPointFeatureVector2D& featuresVector, int pointIndex);
int GetNumberOfDescriptorComponents(const VisualPointFeatureVector2D& featuresVector, int pointIndex);
float GetDescriptorComponent(const VisualPointFeatureVector2D& featuresVector, int pointIndex, int componentIndex);

BitStream ConvertToBitStream(const VisualPointDescriptor2D& vector);
void ConvertFromBitStream(BitStream bitStream, VisualPointDescriptor2D& vector);

}

#endif // VISUAL_POINT_FEATURE_VECTOR_2D_HPP

/** @} */
