/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup BaseTypesWrapper
 *
 * Wrapper for ASN.1 base types
 *
 * @{
 */

#ifndef BASE_TYPES_HPP
#define BASE_TYPES_HPP

#include <Types/C/Frame.h>
#include <Types/C/VisualPointFeatureVector2D.h>
#include <Types/C/VisualPointFeatureVector3D.h>
#include <Types/C/Pointcloud.h>
#include <Types/C/Point.h>
#include <Types/C/CorrespondenceMap3D.h>
#include "Errors/AssertOnTest.hpp"
#include <cstring>

#include <memory>

namespace BaseTypesWrapper
{

// Types

typedef asn1SccT_Int64 T_Int64;
typedef asn1SccT_UInt64 T_UInt64;
typedef asn1SccT_UInt32 T_UInt32;
typedef asn1SccT_UInt16 T_UInt16;
typedef asn1SccT_Boolean T_Boolean;
typedef asn1SccT_String T_String;
typedef asn1SccT_Float T_Float;
typedef asn1SccT_Double T_Double;

// Constants

const int MAX_STRING_SIZE = maxSize_T_String;

// Point Types
struct Point3D
    {
    T_Double x;
    T_Double y;
    T_Double z;
    };

struct Point2D
    {
    T_Double x;
    T_Double y;
    };

// Pointer types

typedef Point2D* Point2DPtr;
typedef Point2D const* Point2DConstPtr;
typedef std::shared_ptr<Point2D> Point2DSharedPtr;
typedef std::shared_ptr<const Point2D> Point2DSharedConstPtr;

//BitStream Allocators Helper functions

void AllocateBitStreamBufferForEncoding(BitStream& bitStream, long size);
void PrepareBitStreamBufferForDeconding(BitStream& bitStream, long size);
void DeallocateBitStreamBuffer(BitStream& bitStream);

template <typename T>
BitStream ConvertToBitStream(const T& inputData, long bitStreamSize, bool (*encodeMethod) (const T*, BitStream*, int*, bool) )
	{
	BitStream bitStream;
	AllocateBitStreamBufferForEncoding(bitStream, bitStreamSize );

	int errorCode = 0;
	bool success = encodeMethod(&inputData, &bitStream, &errorCode, true);

	ASSERT_ON_TEST(success && (errorCode == 0), "Error while executing conversion to bitstream");
	return bitStream;
	}

template <typename T>
void ConvertFromBitStream(BitStream& inputBitStream, long bitStreamSize, T& outputData, bool (*decodeMethod) (T*, BitStream*, int*) )
	{
	PrepareBitStreamBufferForDeconding(inputBitStream, bitStreamSize);
	int errorCode = 0;
	bool success = decodeMethod(&outputData, &inputBitStream, &errorCode);
	ASSERT_ON_TEST(success && (errorCode == 0), "Error while executing conversion from bitstream");
	}

// String manipulation helper functions

void CopyString(const asn1SccT_String& source, asn1SccT_String& destination);
/**
* Copy the content of a std::string to an asn1SccT_String.
* Assume null terminating string in the buffer.
*/
void CopyString(const std::string& source, asn1SccT_String& destination);
/**
 * Copy the content of a asn1SccT_String to an std::string.
 * Assume null terminating string in the buffer.
 */
void CopyString(const asn1SccT_String& source, std::string& destination);
}
#endif // BASE_TYPES_HPP

/** @} */
