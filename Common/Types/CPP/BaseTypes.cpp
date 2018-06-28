/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup BaseTypesWrapper
 *
 * Helper method implementations for BaseTypes
 *
 * @{
 */

#include <BaseTypes.hpp>

namespace BaseTypesWrapper
{

std::allocator<byte> BitStreamAllocator::allocator;

BitStream BitStreamAllocator::AllocateBitStream(long size)
	{
	BitStream bitStream;
	byte* buffer = allocator.allocate(size);
	BitStream_Init(&bitStream, buffer, size);	
	return bitStream;
	}

void BitStreamAllocator::DeallocateBitStream(BitStream bitStream)
	{
	allocator.deallocate(bitStream.buf, bitStream.count);
	}

}

/** @} */
