/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector2D.cpp
 * @date 08/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CppTypes
 * 
 * Implementation of VisualPointFeatureVector2D class.
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
#include "VisualPointFeatureVector2D.hpp"
#include <Errors/Assert.hpp>

namespace CppTypes
{


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

VisualPointFeatureVector2D::VisualPointFeatureVector2D()
	{
	ClearPoints();
	}

VisualPointFeatureVector2D::~VisualPointFeatureVector2D()
	{

	}

void VisualPointFeatureVector2D::AddPoint(uint16_t x, uint16_t y)
	{
	ASSERT(featuresVector.nCount < static_cast<int>(MAX_FEATURE_2D_POINTS), "Features descriptor vector maximum capacity has been reached");
	int currentIndex = featuresVector.nCount;
	featuresVector.arr[currentIndex].point.x = x;
	featuresVector.arr[currentIndex].point.y = y;	
	featuresVector.arr[currentIndex].descriptor.nCount = 0;
	featuresVector.nCount++;
	}

void VisualPointFeatureVector2D::ClearPoints()
	{
	featuresVector.nCount = 0;
	}

int VisualPointFeatureVector2D::GetNumberOfPoints() const
	{
	return featuresVector.nCount;
	}

int VisualPointFeatureVector2D::GetXCoordinate(int pointIndex) const
	{
	ASSERT(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].point.x;
	}

int VisualPointFeatureVector2D::VisualPointFeatureVector2D::GetYCoordinate(int pointIndex) const
	{
	ASSERT(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].point.y;
	}

void VisualPointFeatureVector2D::AddDescriptorComponent(int pointIndex, float component)
	{
	ASSERT(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	ASSERT(featuresVector.arr[pointIndex].descriptor.nCount < static_cast<int>(MAX_DESCRIPTOR_2D_LENGTH), "Descriptor maximum capacity has been reached");
	int currentIndex = featuresVector.arr[pointIndex].descriptor.nCount;
	featuresVector.arr[pointIndex].descriptor.arr[currentIndex] = component;
	featuresVector.arr[pointIndex].descriptor.nCount++;
	}

void VisualPointFeatureVector2D::ClearDescriptor(int pointIndex)
	{
	ASSERT(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	featuresVector.arr[pointIndex].descriptor.nCount = 0;
	}

int VisualPointFeatureVector2D::GetNumberOfDescriptorComponents(int pointIndex) const
	{
	ASSERT(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].descriptor.nCount;
	}

float VisualPointFeatureVector2D::GetDescriptorComponent(int pointIndex, int componentIndex) const
	{
	ASSERT(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	ASSERT(componentIndex < featuresVector.arr[pointIndex].descriptor.nCount, "A missing descriptor component was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].descriptor.arr[componentIndex];
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const T_UInt32 VisualPointFeatureVector2D::MAX_FEATURE_2D_POINTS = CTypes::features2DElementsMax;
const T_UInt32 VisualPointFeatureVector2D::MAX_DESCRIPTOR_2D_LENGTH = CTypes::descriptor2DNameLength;


}

/** @} */
