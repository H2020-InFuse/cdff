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

void AllocateBitStreamBufferForEncoding(BitStream& bitStream, long size)
	{
	byte* buffer = new byte[size];
	BitStream_Init(&bitStream, buffer, size);
	}

void PrepareBitStreamBufferForDeconding(BitStream& bitStream, long size)
	{
	BitStream_AttachBuffer(&bitStream, bitStream.buf, size);
	}

void DeallocateBitStreamBuffer(BitStream& bitStream)
	{
	if (bitStream.buf != NULL)
		{
		delete [] bitStream.buf;
		bitStream.buf = NULL;
		}
	}

}

/** @} */
