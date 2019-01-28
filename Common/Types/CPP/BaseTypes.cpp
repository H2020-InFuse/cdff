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

#include "BaseTypes.hpp"

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


void CopyString(const asn1SccT_String& source, asn1SccT_String& destination)
{
    ASSERT_ON_TEST(source.nCount < MAX_STRING_SIZE, "String size exceeds limits");
    destination.nCount = source.nCount;
    for(int index = 0; index < source.nCount; index++)
    {
        destination.arr[index] = source.arr[index];
    }
}

void CopyString(const T_String& source, std::string& destination)
{
    ASSERT_ON_TEST(source.nCount < MAX_STRING_SIZE, "String size exceeds limits");
    destination = std::string(reinterpret_cast<char const *>(source.arr));
}

void CopyString(std::string const&source,  asn1SccT_String &destination)
{
    destination.nCount = static_cast<int>(source.size() +1); // for null character
    ASSERT_ON_TEST(destination.nCount < static_cast<int>(sizeof(destination.arr) / sizeof(destination.arr[0])), "CopyString: Destination size is insufficient for source size");
    memcpy(destination.arr, source.data(), static_cast<size_t>(destination.nCount));
}



}

/** @} */
