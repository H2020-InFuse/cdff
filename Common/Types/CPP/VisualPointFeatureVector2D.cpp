/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup VisualPointFeatureVector2DWrapper
 * @{
 */

#include "VisualPointFeatureVector2D.hpp"
#include "Errors/AssertOnTest.hpp"
#include "BaseTypes.hpp"

using namespace BaseTypesWrapper;

namespace VisualPointFeatureVector2DWrapper
{

void Copy(const VisualPointFeatureVector2D& source, VisualPointFeatureVector2D& destination)
{
	ClearPoints(destination);
	int numberOfPoints = GetNumberOfPoints(source);
	for (int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
	{
		AddPoint(destination, GetXCoordinate(source, pointIndex), GetYCoordinate(source, pointIndex));
		ClearDescriptor(destination, pointIndex);
		int numberOfDescriptorComponents = GetNumberOfDescriptorComponents(source, pointIndex);
		for (int componentIndex = 0; componentIndex < numberOfDescriptorComponents; componentIndex++)
		{
			AddDescriptorComponent(destination, pointIndex, GetDescriptorComponent(source, pointIndex, componentIndex));
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
	featuresVector.arr[currentIndex].point.arr[0] = x;
	featuresVector.arr[currentIndex].point.arr[1] = y;
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
	return featuresVector.arr[pointIndex].point.arr[0];
}

int GetYCoordinate(const VisualPointFeatureVector2D& featuresVector, int pointIndex)
{
	ASSERT_ON_TEST(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].point.arr[1];
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

BitStream ConvertToBitStream(const VisualPointFeatureVector2D& vector)
	{
	return BaseTypesWrapper::ConvertToBitStream(vector, asn1SccVisualPointFeatureVector2D_REQUIRED_BYTES_FOR_ENCODING, asn1SccVisualPointFeatureVector2D_Encode);
	}

void ConvertFromBitStream(BitStream bitStream, VisualPointFeatureVector2D& vector)
	{
	BaseTypesWrapper::ConvertFromBitStream(bitStream, asn1SccVisualPointFeatureVector2D_REQUIRED_BYTES_FOR_ENCODING, vector, asn1SccVisualPointFeatureVector2D_Decode);
	}
}

/** @} */
