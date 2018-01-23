/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector3D.cpp
 * @date 15/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup VisualPointFeatureVector3DWrapper
 * 
 * Implementation of VisualPointFeatureVector3DWrapper functions.
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

namespace VisualPointFeatureVector3DWrapper
{


/* --------------------------------------------------------------------------
 *
 * Functions
 *
 * --------------------------------------------------------------------------
 */
void Copy(const VisualPointFeatureVector3D& source, VisualPointFeatureVector3D& destination)
	{
	ClearPoints(destination);
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(source); pointIndex++)
		{
		AddPoint(destination, GetXCoordinate(source, pointIndex), GetYCoordinate(source, pointIndex), GetZCoordinate(source, pointIndex));
		ClearDescriptor(destination, pointIndex);
		for(int componentIndex = 0; componentIndex < GetNumberOfDescriptorComponents(source, pointIndex); componentIndex++)
			{
			AddDescriptorComponent(destination, pointIndex, GetDescriptorComponent(source, pointIndex, componentIndex) );
			}
		}
	}

void AddPoint(VisualPointFeatureVector3D& featuresVector, float x, float y, float z)
	{
	ASSERT_ON_TEST(featuresVector.nCount < MAX_FEATURE_3D_POINTS, "Features descriptor vector maximum capacity has been reached");
	int currentIndex = featuresVector.nCount;
	featuresVector.arr[currentIndex].point.kind = VISUAL_POINT_COORDINATES;
	featuresVector.arr[currentIndex].point.u.position.x = x;
	featuresVector.arr[currentIndex].point.u.position.y = y;	
	featuresVector.arr[currentIndex].point.u.position.z = z;	
	featuresVector.arr[currentIndex].descriptor.nCount = 0;
	featuresVector.nCount++;
	}

void AddPoint(VisualPointFeatureVector3D& featuresVector, BaseTypesWrapper::T_UInt64 index, BaseTypesWrapper::T_UInt16 pointCloudIdentifier)
	{
	ASSERT_ON_TEST(featuresVector.nCount < MAX_FEATURE_3D_POINTS, "Features descriptor vector maximum capacity has been reached");
	int currentIndex = featuresVector.nCount;
	featuresVector.arr[currentIndex].point.kind = VISUAL_POINT_REFERENCE;
	featuresVector.arr[currentIndex].point.u.reference.index = index;
	featuresVector.arr[currentIndex].point.u.reference.pointCloudIdentifier = pointCloudIdentifier;
	featuresVector.arr[currentIndex].descriptor.nCount = 0;
	featuresVector.nCount++;
	}

void ClearPoints(VisualPointFeatureVector3D& featuresVector)
	{
	featuresVector.nCount = 0;
	}

int GetNumberOfPoints(const VisualPointFeatureVector3D& featuresVector)
	{
	return featuresVector.nCount;
	}

float GetXCoordinate(const VisualPointFeatureVector3D& featuresVector, int pointIndex)
	{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 3D");
	ASSERT_ON_TEST(featuresVector.arr[pointIndex].point.kind == VISUAL_POINT_COORDINATES, "Request for coordinates for a non-coordinates type visual point");
	return featuresVector.arr[pointIndex].point.u.position.x;
	}

float GetYCoordinate(const VisualPointFeatureVector3D& featuresVector, int pointIndex)
	{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 3D");
	ASSERT_ON_TEST(featuresVector.arr[pointIndex].point.kind == VISUAL_POINT_COORDINATES, "Request for coordinates for a non-coordinates type visual point");
	return featuresVector.arr[pointIndex].point.u.position.y;
	}

float GetZCoordinate(const VisualPointFeatureVector3D& featuresVector, int pointIndex)
	{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 3D");
	ASSERT_ON_TEST(featuresVector.arr[pointIndex].point.kind == VISUAL_POINT_COORDINATES, "Request for coordinates for a non-coordinates type visual point");
	return featuresVector.arr[pointIndex].point.u.position.z;
	}

BaseTypesWrapper::T_UInt64 GetReferenceIndex(const VisualPointFeatureVector3D& featuresVector, int pointIndex)
	{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 3D");
	ASSERT_ON_TEST(featuresVector.arr[pointIndex].point.kind == VISUAL_POINT_REFERENCE, "Request for index for a non-reference type visual point");
	return featuresVector.arr[pointIndex].point.u.reference.index;
	}

BaseTypesWrapper::T_UInt16 GetPointCloudIdentifier(const VisualPointFeatureVector3D& featuresVector, int pointIndex)
	{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 3D");
	ASSERT_ON_TEST(featuresVector.arr[pointIndex].point.kind == VISUAL_POINT_REFERENCE, "Request for point cloud id for a non-reference type visual point");
	return featuresVector.arr[pointIndex].point.u.reference.pointCloudIdentifier;
	}

void AddDescriptorComponent(VisualPointFeatureVector3D& featuresVector, int pointIndex, float component)
	{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 3D");
	ASSERT_ON_TEST(featuresVector.arr[pointIndex].descriptor.nCount < MAX_DESCRIPTOR_3D_LENGTH, "Descriptor maximum capacity has been reached");
	int currentIndex = featuresVector.arr[pointIndex].descriptor.nCount;
	featuresVector.arr[pointIndex].descriptor.arr[currentIndex] = component;
	featuresVector.arr[pointIndex].descriptor.nCount++;
	}

void ClearDescriptor(VisualPointFeatureVector3D& featuresVector, int pointIndex)
	{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 3D");
	featuresVector.arr[pointIndex].descriptor.nCount = 0;
	}

int GetNumberOfDescriptorComponents(const VisualPointFeatureVector3D& featuresVector, int pointIndex)
	{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 3D");
	return featuresVector.arr[pointIndex].descriptor.nCount;
	}

float GetDescriptorComponent(const VisualPointFeatureVector3D& featuresVector, int pointIndex, int componentIndex)
	{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 3D");
	ASSERT_ON_TEST(componentIndex < featuresVector.arr[pointIndex].descriptor.nCount, "A missing descriptor component was requested from a features vector 3D");
	return featuresVector.arr[pointIndex].descriptor.arr[componentIndex];
	}

}

/** @} */
