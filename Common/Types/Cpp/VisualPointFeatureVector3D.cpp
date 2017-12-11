/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector3D.cpp
 * @date 08/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CppTypes
 * 
 * Implementation of VisualPointFeatureVector3D class.
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
#include "VisualPointFeatureVector3D.hpp"
#include <Errors/Assert.hpp>

namespace CppTypes
{


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

VisualPointFeatureVector3D::VisualPointFeatureVector3D()
	{
	ClearPoints();
	}

VisualPointFeatureVector3D::~VisualPointFeatureVector3D()
	{

	}

void VisualPointFeatureVector3D::AddPoint(float x, float y, float z)
	{
	ASSERT(featuresVector.nCount < static_cast<int>(MAX_FEATURE_3D_POINTS), "Features descriptor vector maximum capacity has been reached");
	int currentIndex = featuresVector.nCount;
	featuresVector.arr[currentIndex].point.x = x;
	featuresVector.arr[currentIndex].point.y = y;	
	featuresVector.arr[currentIndex].point.z = z;	
	featuresVector.arr[currentIndex].descriptor.nCount = 0;
	featuresVector.nCount++;
	}

void VisualPointFeatureVector3D::ClearPoints()
	{
	featuresVector.nCount = 0;
	}

int VisualPointFeatureVector3D::GetNumberOfPoints() const
	{
	return featuresVector.nCount;
	}

float VisualPointFeatureVector3D::GetXCoordinate(int pointIndex) const
	{
	ASSERT(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].point.x;
	}

float VisualPointFeatureVector3D::GetYCoordinate(int pointIndex) const
	{
	ASSERT(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].point.y;
	}

float VisualPointFeatureVector3D::GetZCoordinate(int pointIndex) const
	{
	ASSERT(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].point.z;
	}

void VisualPointFeatureVector3D::AddDescriptorComponent(int pointIndex, float component)
	{
	ASSERT(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	ASSERT(featuresVector.arr[pointIndex].descriptor.nCount < static_cast<int>(MAX_DESCRIPTOR_3D_LENGTH), "Descriptor maximum capacity has been reached");
	int currentIndex = featuresVector.arr[pointIndex].descriptor.nCount;
	featuresVector.arr[pointIndex].descriptor.arr[currentIndex] = component;
	featuresVector.arr[pointIndex].descriptor.nCount++;
	}

void VisualPointFeatureVector3D::ClearDescriptor(int pointIndex)
	{
	ASSERT(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	featuresVector.arr[pointIndex].descriptor.nCount = 0;
	}

int VisualPointFeatureVector3D::GetNumberOfDescriptorComponents(int pointIndex) const
	{
	ASSERT(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].descriptor.nCount;
	}

float VisualPointFeatureVector3D::GetDescriptorComponent(int pointIndex, int componentIndex) const
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
const T_UInt32 VisualPointFeatureVector3D::MAX_FEATURE_3D_POINTS = CTypes::features3DElementsMax;
const T_UInt32 VisualPointFeatureVector3D::MAX_DESCRIPTOR_3D_LENGTH = CTypes::descriptor3DNameLength;


}

/** @} */
