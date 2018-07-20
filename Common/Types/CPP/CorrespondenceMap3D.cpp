/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup CorrespondenceMap3DWrapper
 * @{
 */

#include "CorrespondenceMap3D.hpp"
#include <Errors/Assert.hpp>

namespace CorrespondenceMap3DWrapper
{

using namespace BaseTypesWrapper;

void Copy(const CorrespondenceMap3D& source, CorrespondenceMap3D& destination)
{
	ClearCorrespondences(destination);
	for (int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(source); correspondenceIndex++)
	{
		AddCorrespondence(
			destination,
			GetSource(source, correspondenceIndex),
			GetSink(source, correspondenceIndex),
			GetProbability(source, correspondenceIndex)
		);
	}
}

CorrespondenceMap3DPtr NewCorrespondenceMap3D()
{
	CorrespondenceMap3DPtr correspondenceMap = new CorrespondenceMap3D();
	Initialize(*correspondenceMap);
	return correspondenceMap;
}

CorrespondenceMap3DSharedPtr NewSharedCorrespondenceMap3D()
{
	CorrespondenceMap3DSharedPtr sharedCorrespondenceMap = std::make_shared<CorrespondenceMap3D>();
	Initialize(*sharedCorrespondenceMap);
	return sharedCorrespondenceMap;
}

void Initialize(CorrespondenceMap3D& correspondenceMap)
{
	ClearCorrespondences(correspondenceMap);
}

void AddCorrespondence(CorrespondenceMap3D& correspondenceMap, Point3D source, Point3D sink, T_Float probability)
{
	ASSERT_ON_TEST(correspondenceMap.nCount < MAX_CORRESPONDENCES_3D, "Correspondence Map 3D maximum capacity has been reached");
	int currentIndex = correspondenceMap.nCount;
	correspondenceMap.arr[currentIndex].source = source;
	correspondenceMap.arr[currentIndex].sink = sink;
	correspondenceMap.arr[currentIndex].probability = probability;
	correspondenceMap.nCount++;
}

void ClearCorrespondences(CorrespondenceMap3D& correspondenceMap)
{
	correspondenceMap.nCount = 0;
}

int GetNumberOfCorrespondences(const CorrespondenceMap3D& correspondenceMap)
{
	return correspondenceMap.nCount;
}

Point3D GetSource(const CorrespondenceMap3D& correspondenceMap, const int correspondenceIndex)
{
	ASSERT_ON_TEST(correspondenceIndex < correspondenceMap.nCount, "A missing correspondence was requested from a correspondence map 3d");
	return correspondenceMap.arr[correspondenceIndex].source;
}

Point3D GetSink(const CorrespondenceMap3D& correspondenceMap, const int correspondenceIndex)
{
	ASSERT_ON_TEST(correspondenceIndex < correspondenceMap.nCount, "A missing correspondence was requested from a correspondence map 3d");
	return correspondenceMap.arr[correspondenceIndex].sink;
}

T_Float GetProbability(const CorrespondenceMap3D& correspondenceMap, const int correspondenceIndex)
{
	ASSERT_ON_TEST(correspondenceIndex < correspondenceMap.nCount, "A missing correspondence was requested from a correspondence map 3d");
	return correspondenceMap.arr[correspondenceIndex].probability;
}

BitStream ConvertToBitStream(const CorrespondenceMap3D& map)
	CONVERT_TO_BIT_STREAM(map, asn1SccCorrespondenceMap3D_REQUIRED_BYTES_FOR_ENCODING, asn1SccCorrespondenceMap3D_Encode)

void ConvertFromBitStream(BitStream bitStream, CorrespondenceMap3D& map)
	CONVERT_FROM_BIT_STREAM(bitStream, asn1SccCorrespondenceMap3D_REQUIRED_BYTES_FOR_ENCODING, map, asn1SccCorrespondenceMap3D_Decode)

}

/** @} */
