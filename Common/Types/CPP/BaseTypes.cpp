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
#include <Errors/Assert.hpp>

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

void BitStreamAllocator::PrepareBitStreamForDecoding(BitStream& bitStream, long size)
	{
	BitStream_AttachBuffer(&bitStream, bitStream.buf, size);
	}

void BitStreamAllocator::DeallocateBitStream(BitStream& bitStream, long size)
	{
	allocator.deallocate(bitStream.buf, size);
	}

}

/** @} */
