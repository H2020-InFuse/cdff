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

std::allocator<unsigned char> BitStreamAllocator::allocator;

BitStream BitStreamAllocator::AllocateBitStream(long size)
	{
	BitStream bitStream;
	unsigned char* buffer = allocator.allocate(size);
	BitStream_Init(&bitStream, buffer, size);	
	return bitStream;
	}

void BitStreamAllocator::DeallocateBitStream(BitStream bitStream)
	{
	allocator.deallocate(bitStream.buf, bitStream.count);
	}

}

/** @} */
