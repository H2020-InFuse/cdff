/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup VisualPointFeatureVector3DWrapper
 * @{
 */

#include "VisualPointFeatureVector3D.hpp"
#include <Errors/Assert.hpp>
#include <BaseTypes.hpp>

using namespace BaseTypesWrapper;

namespace VisualPointFeatureVector3DWrapper
{

void Copy(const VisualPointFeatureVector3D& source, VisualPointFeatureVector3D& destination)
{
	ClearPoints(destination);
	for (int pointIndex = 0; pointIndex < GetNumberOfPoints(source); pointIndex++)
	{
		if (GetPointType(source, pointIndex) == VISUAL_POINT_POSITION)
		{
			AddPoint(destination, GetXCoordinate(source, pointIndex), GetYCoordinate(source, pointIndex), GetZCoordinate(source, pointIndex));
		}
		else if (GetPointType(source, pointIndex) == VISUAL_POINT_REFERENCE)
		{
			AddPoint(destination, GetReferenceIndex(source, pointIndex), GetPointCloudIdentifier(source, pointIndex));
		}

		if (GetPointType(source, pointIndex) != VISUAL_POINT_NONE)
		{
			ClearDescriptor(destination, pointIndex);
			for (int componentIndex = 0; componentIndex < GetNumberOfDescriptorComponents(source, pointIndex); componentIndex++)
			{
				AddDescriptorComponent(destination, pointIndex, GetDescriptorComponent(source, pointIndex, componentIndex));
			}
		}
	}
}

VisualPointFeatureVector3DPtr NewVisualPointFeatureVector3D()
{
	VisualPointFeatureVector3DPtr vector = new VisualPointFeatureVector3D();
	Initialize(*vector);
	return vector;
}

VisualPointFeatureVector3DSharedPtr NewSharedVisualPointFeatureVector3D()
{
	VisualPointFeatureVector3DSharedPtr sharedVector = std::make_shared<VisualPointFeatureVector3D>();
	Initialize(*sharedVector);
	return sharedVector;
}

void Initialize(VisualPointFeatureVector3D& featuresVector)
{
	ClearPoints(featuresVector);
}

void AddPoint(VisualPointFeatureVector3D& featuresVector, float x, float y, float z)
{
	ASSERT_ON_TEST(featuresVector.list.nCount < MAX_FEATURE_3D_POINTS, "Features descriptor vector maximum capacity has been reached");
	int currentIndex = featuresVector.list.nCount;
	if (currentIndex == 0)
	{
		featuresVector.list_type = ALL_POSITIONS_VECTOR;
	}
	else if (featuresVector.list_type != ALL_POSITIONS_VECTOR)
	{
		featuresVector.list_type = HYBRID_VECTOR;
	}

	featuresVector.list.arr[currentIndex].point.kind = VisualPoint::position_PRESENT;
	featuresVector.list.arr[currentIndex].point.u.position.x = x;
	featuresVector.list.arr[currentIndex].point.u.position.y = y;
	featuresVector.list.arr[currentIndex].point.u.position.z = z;
	featuresVector.list.arr[currentIndex].descriptor.nCount = 0;
	featuresVector.list.nCount++;
}

void AddPoint(VisualPointFeatureVector3D& featuresVector, BaseTypesWrapper::T_UInt64 index, BaseTypesWrapper::T_UInt16 pointCloudIdentifier)
{
	ASSERT_ON_TEST(featuresVector.list.nCount < MAX_FEATURE_3D_POINTS, "Features descriptor vector maximum capacity has been reached");
	int currentIndex = featuresVector.list.nCount;
	if (currentIndex == 0)
	{
		featuresVector.list_type = ALL_REFERENCES_VECTOR;
	}
	else if (featuresVector.list_type != ALL_REFERENCES_VECTOR)
	{
		featuresVector.list_type = HYBRID_VECTOR;
	}

	featuresVector.list.arr[currentIndex].point.kind = VisualPoint::reference_PRESENT;
	featuresVector.list.arr[currentIndex].point.u.reference.index = index;
	featuresVector.list.arr[currentIndex].point.u.reference.pointCloudIdentifier = pointCloudIdentifier;
	featuresVector.list.arr[currentIndex].descriptor.nCount = 0;
	featuresVector.list.nCount++;
}

void ClearPoints(VisualPointFeatureVector3D& featuresVector)
{
	featuresVector.list.nCount = 0;
}

VisualPointFeatureVector3DType GetVectorType(const VisualPointFeatureVector3D& featuresVector)
{
	return featuresVector.list_type;
}

int GetNumberOfPoints(const VisualPointFeatureVector3D& featuresVector)
{
	return featuresVector.list.nCount;
}

float GetXCoordinate(const VisualPointFeatureVector3D& featuresVector, int pointIndex)
{
	ASSERT_ON_TEST(pointIndex < featuresVector.list.nCount, "A missing point was requested from a features vector 3D");
	ASSERT_ON_TEST(featuresVector.list.arr[pointIndex].point.kind == VisualPoint::position_PRESENT, "Request for coordinates for a non-coordinates type visual point");
	return featuresVector.list.arr[pointIndex].point.u.position.x;
}

float GetYCoordinate(const VisualPointFeatureVector3D& featuresVector, int pointIndex)
{
	ASSERT_ON_TEST(pointIndex < featuresVector.list.nCount, "A missing point was requested from a features vector 3D");
	ASSERT_ON_TEST(featuresVector.list.arr[pointIndex].point.kind == VisualPoint::position_PRESENT, "Request for coordinates for a non-coordinates type visual point");
	return featuresVector.list.arr[pointIndex].point.u.position.y;
}

float GetZCoordinate(const VisualPointFeatureVector3D& featuresVector, int pointIndex)
{
	ASSERT_ON_TEST(pointIndex < featuresVector.list.nCount, "A missing point was requested from a features vector 3D");
	ASSERT_ON_TEST(featuresVector.list.arr[pointIndex].point.kind == VisualPoint::position_PRESENT, "Request for coordinates for a non-coordinates type visual point");
	return featuresVector.list.arr[pointIndex].point.u.position.z;
}

BaseTypesWrapper::T_UInt64 GetReferenceIndex(const VisualPointFeatureVector3D& featuresVector, int pointIndex)
{
	ASSERT_ON_TEST(pointIndex < featuresVector.list.nCount, "A missing point was requested from a features vector 3D");
	ASSERT_ON_TEST(featuresVector.list.arr[pointIndex].point.kind == VisualPoint::reference_PRESENT, "Request for index for a non-reference type visual point");
	return featuresVector.list.arr[pointIndex].point.u.reference.index;
}

BaseTypesWrapper::T_UInt16 GetPointCloudIdentifier(const VisualPointFeatureVector3D& featuresVector, int pointIndex)
{
	ASSERT_ON_TEST(pointIndex < featuresVector.list.nCount, "A missing point was requested from a features vector 3D");
	ASSERT_ON_TEST(featuresVector.list.arr[pointIndex].point.kind == VisualPoint::reference_PRESENT, "Request for point cloud id for a non-reference type visual point");
	return featuresVector.list.arr[pointIndex].point.u.reference.pointCloudIdentifier;
}

VisualPointType GetPointType(const VisualPointFeatureVector3D& featuresVector, int pointIndex)
{
	ASSERT_ON_TEST(pointIndex < featuresVector.list.nCount, "A missing point was requested from a features vector 3D");
	switch(featuresVector.list.arr[pointIndex].point.kind)
	{
		case VisualPoint::VisualPointFeature3D_point_NONE: return VISUAL_POINT_NONE;
		case VisualPoint::position_PRESENT: return VISUAL_POINT_POSITION;
		case VisualPoint::reference_PRESENT: return VISUAL_POINT_REFERENCE;
		default: ASSERT(false, "Unhandled point type in VisualPointFeatureVector3D");
	}
	return VISUAL_POINT_NONE;
}

void AddDescriptorComponent(VisualPointFeatureVector3D& featuresVector, int pointIndex, float component)
{
	ASSERT_ON_TEST(pointIndex < featuresVector.list.nCount, "A missing point was requested from a features vector 3D");
	ASSERT_ON_TEST(featuresVector.list.arr[pointIndex].descriptor.nCount < MAX_DESCRIPTOR_3D_LENGTH, "Descriptor maximum capacity has been reached");
	int currentIndex = featuresVector.list.arr[pointIndex].descriptor.nCount;
	featuresVector.list.arr[pointIndex].descriptor.arr[currentIndex] = component;
	featuresVector.list.arr[pointIndex].descriptor.nCount++;
}

void ClearDescriptor(VisualPointFeatureVector3D& featuresVector, int pointIndex)
{
	ASSERT_ON_TEST(pointIndex < featuresVector.list.nCount, "A missing point was requested from a features vector 3D");
	featuresVector.list.arr[pointIndex].descriptor.nCount = 0;
}

int GetNumberOfDescriptorComponents(const VisualPointFeatureVector3D& featuresVector, int pointIndex)
{
	ASSERT_ON_TEST(pointIndex < featuresVector.list.nCount, "A missing point was requested from a features vector 3D");
	return featuresVector.list.arr[pointIndex].descriptor.nCount;
}

float GetDescriptorComponent(const VisualPointFeatureVector3D& featuresVector, int pointIndex, int componentIndex)
{
	ASSERT_ON_TEST(pointIndex < featuresVector.list.nCount, "A missing point was requested from a features vector 3D");
	ASSERT_ON_TEST(componentIndex < featuresVector.list.arr[pointIndex].descriptor.nCount, "A missing descriptor component was requested from a features vector 3D");
	return featuresVector.list.arr[pointIndex].descriptor.arr[componentIndex];
}

BitStream ConvertToBitStream(const VisualPointFeatureVector3D& vector)
	{
	BitStream bitStream = BitStreamAllocator::AllocateBitStream( sizeof(VisualPointFeatureVector3D) );
	int errorCode;
	bool success = asn1SccVisualPointFeatureVector3D_Encode(&vector, &bitStream, &errorCode, true);

	ASSERT(success, "Error while converting VisualPointFeatureVector3D to BitStream");
	return bitStream;
	}

void ConvertFromBitStream(BitStream bitStream, VisualPointFeatureVector3D& vector)
	{
	int errorCode;
	bool success = asn1SccVisualPointFeatureVector3D_Decode(&vector, &bitStream, &errorCode);
	ASSERT(success, "Error while converting BitStream to VisualPointFeatureVector3D");
	}

}

/** @} */
