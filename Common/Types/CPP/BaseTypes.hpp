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

#include <Frame.h>
#include <VisualPointFeatureVector2D.h>
#include <VisualPointFeatureVector3D.h>
#include <Pointcloud.h>
#include <Point.h>
#include <CorrespondenceMap3D.h>

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

#define CONVERT_TO_BIT_STREAM(inputData, bitStreamSize, encodeMethod) \
	{ \
	BitStream bitStream; \
	AllocateBitStreamBufferForEncoding(bitStream, bitStreamSize ); \
	\
	int errorCode = 0; \
	bool success = encodeMethod(&inputData, &bitStream, &errorCode, true); \
	\
	ASSERT(success && (errorCode == 0), "Error while executing #conversionMethod"); \
	return bitStream; \
	}

#define CONVERT_FROM_BIT_STREAM(inputBitStream, bitStreamSize, outputData, decodeMethod) \
	{ \
	PrepareBitStreamBufferForDeconding(inputBitStream, bitStreamSize); \
	int errorCode = 0; \
	bool success = decodeMethod(&outputData, &inputBitStream, &errorCode); \
	ASSERT(success && (errorCode == 0), "Error while executing #conversionMethod"); \
	}


// String manipulation helper functions

void CopyString(const asn1SccT_String& source, asn1SccT_String& destination);
}


#endif // BASE_TYPES_HPP

/** @} */
