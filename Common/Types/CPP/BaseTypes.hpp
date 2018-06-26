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
#include <Geometry.h>
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
typedef asn1SccPoint3D Point3D;
typedef asn1SccPoint2D Point2D;

// Pointer types

typedef Point2D* Point2DPtr;
typedef Point2D const* Point2DConstPtr;
typedef std::shared_ptr<Point2D> Point2DSharedPtr;
typedef std::shared_ptr<const Point2D> Point2DSharedConstPtr;

// BitStream Allocator
class BitStreamAllocator
	{
	public:
		static BitStream AllocateBitStream(long size);
		static void DeallocateBitStream(BitStream bitStream);

	private:
		static std::allocator<unsigned char> allocator;
	};

}

#endif // BASE_TYPES_HPP

/** @} */
