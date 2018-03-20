/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector2D.cpp
 * @date 15/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup VisualPointFeatureVector2DWrapper
 * 
 * Implementation of VisualPointFeatureVector2DWrapper functions.
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

namespace VisualPointFeatureVector2DWrapper
{


/* --------------------------------------------------------------------------
 *
 * Functions
 *
 * --------------------------------------------------------------------------
 */

void Copy(const VisualPointFeatureVector2D& source, VisualPointFeatureVector2D& destination)
	{
	ClearPoints(destination);
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(source); pointIndex++)
		{
		AddPoint(destination, GetXCoordinate(source, pointIndex), GetYCoordinate(source, pointIndex) );
		ClearDescriptor(destination, pointIndex);
		for(int componentIndex = 0; componentIndex < GetNumberOfDescriptorComponents(source, pointIndex); componentIndex++)
			{
			AddDescriptorComponent(destination, pointIndex, GetDescriptorComponent(source, pointIndex, componentIndex) );
			}
		}
	}

VisualPointFeatureVector2DPtr NewVisualPointFeatureVector2D()
	{
	VisualPointFeatureVector2DPtr vector = new VisualPointFeatureVector2D();
	Initialize(*vector);
	return vector;
	}

VisualPointFeatureVector2DSharedPtr NewSharedVisualPointFeatureVector2D()
	{
	VisualPointFeatureVector2DSharedPtr sharedVector = std::make_shared<VisualPointFeatureVector2D>();
	Initialize(*sharedVector);
	return sharedVector;
	}

void Initialize(VisualPointFeatureVector2D& featuresVector)
	{
	ClearPoints(featuresVector);
	}

void AddPoint(VisualPointFeatureVector2D& featuresVector, uint16_t x, uint16_t y)
	{
	ASSERT_ON_TEST(featuresVector.nCount < MAX_FEATURE_2D_POINTS, "Features descriptor vector maximum capacity has been reached");
	int currentIndex = featuresVector.nCount;
	featuresVector.arr[currentIndex].point.x = x;
	featuresVector.arr[currentIndex].point.y = y;	
	featuresVector.arr[currentIndex].descriptor.nCount = 0;
	featuresVector.nCount++;
	}

void ClearPoints(VisualPointFeatureVector2D& featuresVector)
	{
	featuresVector.nCount = 0;
	}

int GetNumberOfPoints(const VisualPointFeatureVector2D& featuresVector)
	{
	return featuresVector.nCount;
	}

int GetXCoordinate(const VisualPointFeatureVector2D& featuresVector, int pointIndex)
	{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].point.x;
	}

int GetYCoordinate(const VisualPointFeatureVector2D& featuresVector, int pointIndex)
	{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].point.y;
	}

void AddDescriptorComponent(VisualPointFeatureVector2D& featuresVector, int pointIndex, float component)
	{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	ASSERT_ON_TEST(featuresVector.arr[pointIndex].descriptor.nCount < MAX_DESCRIPTOR_2D_LENGTH, "Descriptor maximum capacity has been reached");
	int currentIndex = featuresVector.arr[pointIndex].descriptor.nCount;
	featuresVector.arr[pointIndex].descriptor.arr[currentIndex] = component;
	featuresVector.arr[pointIndex].descriptor.nCount++;
	}

void ClearDescriptor(VisualPointFeatureVector2D& featuresVector, int pointIndex)
	{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	featuresVector.arr[pointIndex].descriptor.nCount = 0;
	}

int GetNumberOfDescriptorComponents(const VisualPointFeatureVector2D& featuresVector, int pointIndex)
	{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].descriptor.nCount;
	}

float GetDescriptorComponent(const VisualPointFeatureVector2D& featuresVector, int pointIndex, int componentIndex)
	{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	ASSERT_ON_TEST(componentIndex < featuresVector.arr[pointIndex].descriptor.nCount, "A missing descriptor component was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].descriptor.arr[componentIndex];
	}

}

/** @} */
