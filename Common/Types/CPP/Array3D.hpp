/**
 * @author Vincent Bissonnette
 */

/**
 * @addtogroup Array3DWrapper
 *
 * Wrapper for ASN.1 Array3D type
 *
 * @{
 */

#ifndef ARRAY3D_HPP
#define ARRAY3D_HPP

#include <Array3D.h>
#include <taste-extended.h>

#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>
#include "Errors/Assert.hpp"
#include <cstring>

/**
 *
 */
namespace Array3DWrapper
{

// Types
typedef asn1SccArray3D_depth_t Array3DDepth;
typedef asn1SccArray3D_data Array3DData;
typedef asn1SccArray3D Array3D;

// Enumerated types

const Array3DDepth ARRAY3D_8U = asn1Sccdepth_8U;
const Array3DDepth ARRAY3D_8S = asn1Sccdepth_8S;
const Array3DDepth ARRAY3D_16U = asn1Sccdepth_16U;
const Array3DDepth ARRAY3D_16S = asn1Sccdepth_16S;
const Array3DDepth ARRAY3D_32S = asn1Sccdepth_32S;
const Array3DDepth ARRAY3D_32F = asn1Sccdepth_32F;
const Array3DDepth ARRAY3D_64F = asn1Sccdepth_64F;


// Global constant variables

const int ARRAY3D_VERSION = array3D_Version;
const int MAX_ARRAY3D_BYTE_SIZE = array3DMaxBytes;
const int MAX_DATA_ROWS = array3DMaxRows;
const int MAX_DATA_COLUMNS = array3DMaxCols;
const int MAX_DATA_CHANNELS = array3DMaxChannels;

// Pointer types

typedef Array3D* Array3DPtr;
typedef Array3D const* Array3DConstPtr;
typedef std::shared_ptr<Array3D> Array3DSharedPtr;
typedef std::shared_ptr<const Array3D> Array3DSharedConstPtr;

// Functions

Array3DPtr NewArray3D();
Array3DSharedPtr NewSharedArray3D();
Array3DConstPtr Clone(const Array3D& source);
Array3DSharedPtr SharedClone(const Array3D& source);

/**
 * Initialize the fields of the frame to sane default values.
 * @param frame The frame to initialize
 */
void Initialize(Array3D& array);

/**
 * Copy the data and the attributes of the source array to the destination. This
 * does not invalidate the source array.
 *
 * @param source The array from which to copy the data and attributes
 * @param destination The array into which to copy the data and attributes
 */
void Copy(const Array3D& source, Array3D& destination);

void SetArray3DDepth(Array3D& array, Array3DDepth arrayDepth);
Array3DDepth GetArray3DDepth(const Array3D& array);

void SetArray3DSize(Array3D& array, BaseTypesWrapper::T_UInt32 cols, BaseTypesWrapper::T_UInt32 rows);
BaseTypesWrapper::T_UInt32 GetArray3DCols(const Array3D& array);
BaseTypesWrapper::T_UInt32 GetArray3DRows(const Array3D& array);

void SetArray3DChannels(Array3D& array, BaseTypesWrapper::T_UInt32 channels);
BaseTypesWrapper::T_UInt32 GetArray3DChannels(const Array3D& array);

void SetArray3DRowSize(Array3D& array, BaseTypesWrapper::T_UInt32 rowSize);
BaseTypesWrapper::T_UInt32 GetArray3DRowSize(const Array3D& array);


void ClearData(Array3D& array);
byte GetDataByte(const Array3D& array, int index);
int GetNumberOfDataBytes(const Array3D& array);

BitStream ConvertToBitStream(const Array3D& array);
void ConvertFromBitStream(BitStream bitStream, Array3D& array);

    /* !! ASSUMES Little Endian */
    template<typename T>
    void AppendData(Array3D &array, T data)
    {
        ASSERT_ON_TEST(array.data.nCount + static_cast<int>(sizeof(T)) < MAX_ARRAY3D_BYTE_SIZE, "Array data will exceed limits");
        std::memcpy(&array.data.arr[array.data.nCount], &data, sizeof (T));
        array.data.nCount+= sizeof (T);
    }
}

#endif // ARRAY3D_HPP

/** @} */
