/**
 * @author Vincent Bissonnette
 */

/**
 * @addtogroup Array3DWrapper
 * @{
 */

#include "Array3D.hpp"

namespace Array3DWrapper
{

using namespace BaseTypesWrapper;

    void CopyArray3DData(const Array3D& source, Array3D& destination)
	{
		// copy buffers
        ASSERT_ON_TEST(source.data.nCount < MAX_ARRAY3D_BYTE_SIZE, "Array3D data exceeds limits");
        std::memcpy(&destination.data.arr, &source.data.arr, source.data.nCount);
        destination.data.nCount = source.data.nCount;
	}

void Copy(const Array3D& source, Array3D& destination)
{
	destination.msgVersion = source.msgVersion;
    destination.rows = source.rows;
    destination.cols = source.cols;
    destination.channels = source.channels;
    destination.depth = source.depth;
    destination.rowSize = source.rowSize;

    CopyArray3DData(source, destination);
}

Array3DPtr NewArray3D()
{
    Array3DPtr array = new Array3D();
    Initialize(*array);
    return array;
}

Array3DSharedPtr NewSharedArray3D()
{
    Array3DSharedPtr sharedArray = std::make_shared<Array3D>();
    Initialize(*sharedArray);
    return sharedArray;
}

Array3DConstPtr Clone(const Array3D& source)
{
    Array3DPtr array = new Array3D();
    Copy(source, *array);
    return array;
}

Array3DSharedPtr SharedClone(const Array3D& source)
{
    Array3DSharedPtr sharedArray = std::make_shared<Array3D>();
    Copy(source, *sharedArray);
    return sharedArray;
}

void Initialize(Array3D& array)
{
array.data.nCount = 0;
array.rows = 0;
array.cols = 0;
array.channels = 0;
array.depth = ARRAY3D_8U;
array.rowSize = 0;
}

void SetArray3DDepth(Array3D& array, Array3DDepth arrayDepth)
{
    array.depth = arrayDepth;
}

Array3DDepth GetArray3DDepth(const Array3D& array)
{
    return array.depth;
}

void SetArray3DSize(Array3D& array, T_UInt32 cols, T_UInt32 rows)
{
    array.cols = cols;
    array.rows = rows;
}

T_UInt32 GetArray3DCols(const Array3D& array)
{
    return array.cols;
}
T_UInt32 GetArray3DRows(const Array3D& array)
{
    return array.rows;
}

void SetArray3DChannels(Array3D& array, T_UInt32 channels)
{
    array.channels = channels;
}

T_UInt32 GetArray3DChannels(const Array3D& array)
{
    return array.channels;
}

void SetArray3DRowSize(Array3D& array, T_UInt32 rowSize)
{
    array.rowSize = rowSize;
}

T_UInt32 GetArray3DRowSize(const Array3D& array)
{
    return array.rowSize;
}

void ClearData(Array3D& array)
{
    array.data.nCount = 0;
}

byte GetDataByte(const Array3D& array, int index)
{
    ASSERT_ON_TEST(index < array.data.nCount, "Requesting missing array data");
    return array.data.arr[index];
}

int GetNumberOfDataBytes(const Array3D& array)
{
    return array.data.nCount;
}

BitStream ConvertToBitStream(const Array3D& array)
	{
	return BaseTypesWrapper::ConvertToBitStream(array, asn1SccArray3D_REQUIRED_BYTES_FOR_ENCODING, asn1SccArray3D_Encode);
	}

void ConvertFromBitStream(BitStream bitStream, Array3D& array)
	{
	BaseTypesWrapper::ConvertFromBitStream(bitStream, asn1SccArray3D_REQUIRED_BYTES_FOR_ENCODING, array, asn1SccArray3D_Decode);
	}
}

/** @} */
