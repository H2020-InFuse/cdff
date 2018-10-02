/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup FrameWrapper
 *
 * Wrapper for ASN.1 Frame type
 *
 * @{
 */

#ifndef FRAME_HPP
#define FRAME_HPP

#include <Frame.h>
#include <Array3D.h>
#include <Array3D.hpp>
#include <taste-extended.h>

#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>
#include "Errors/Assert.hpp"
#include <cstring>
#include <Types/C/Frame.h>

/**
 *  The `Frame` type is the C++ interface to the compiled ASN.1 Frame. A
 *  Frame is a struct composed of the following attributes:
 *
 *   - `frame-time`, the time at which the Frame was created,
 *   - `received-time`, the time at which the Frame was received,
 *   - `image`, the actual image data stored in a `FrameImage`,
 *   - `attributes`, additional information about the frame (_e.g._ `isHDR`)
 *   - `datasize`, the size of the frame in pixels,
 *   - `data-depth`, the number of bits per channel,
 *   - `pixel-size`, the size of a single pixel in bytes. This is
 *     `number_of_channels * bytes_per_channel (data-depth)`,
 *   - `row-size`, the size of a row in bytes, this is `pixel-size *
 *     datasize.width`. This value will be inaccurate for compressed image modes
 *     since they use a varying number of bytes per pixel,
 *   - `frame-mode`, the type of pixels (RGB, BGR, Grayscale, ...),
 *   - `frame-status`, whether the frame has valid data.
 *
 *  The data buffer is statically allocated. This makes the Frame object quite
 *  heavy and causes it o not be stack allocatable. Creating a `Frame` as:
 *
 *  ```
 *  Frame new_frame;  // SEGFAULT !!
 *  ```
 *
 *  Will fail with a hard to debug segfault before entering the function. You
 *  should always use dynamic allocation for Frames using either the raw or the
 *  shared pointers (or their `const` variants):
 *
 *  ```
 *  FramePtr rawFrame = NewFrame();
 *  FrameSharedPtr sharedFrame = NewSharedFrame();
 *  ```
 *
 *  You can copy from an existing `Frame` with the `FrameWrapper::Copy` function
 *  or initialize a new frame with the `FrameWrapper::Initialize` function.
 *
 *  This documentation was sourced from the documentation of the
 *  [ROCK base types](https://github.com/rock-core/base-types/blob/75eb9fde79c803408bc0d6068839a21495590b0b/src/samples/Frame.hpp)
 *
 */
namespace FrameWrapper
{

// Types

typedef asn1SccFrame_error_t FrameError;
typedef asn1SccFrame_metadata_t_errValues FrameMetadataError;
typedef asn1SccFrame_attrib_t FrameAttribute;
typedef asn1SccFrame_metadata_t_attributes FrameMetadataAttributeList;
typedef asn1SccFrame_extrinsic_t FrameExtrinsic;
typedef asn1SccFrame_pixelModel_t FramePixelModel;
typedef asn1SccFrame_mode_t FrameMode;
typedef asn1SccFrame_status_t FrameStatus;
typedef asn1SccFrame_metadata_t FrameMetadata;
typedef asn1SccFrame_cameraModel_t FrameCameraModel;
typedef asn1SccFrame_intrinsic_t FrameIntrinsic;
typedef asn1SccFrame Frame;
typedef asn1SccFramePair FramePair;

// Enumerated types

const FrameMode MODE_UNDEFINED = asn1Sccmode_UNDEF;
const FrameMode MODE_GRAYSCALE = asn1Sccmode_GRAY;
const FrameMode MODE_RGB = asn1Sccmode_RGB;
const FrameMode MODE_RGBA = asn1Sccmode_RGBA;
const FrameMode MODE_BGR = asn1Sccmode_BGR;
const FrameMode MODE_BGRA = asn1Sccmode_BGRA;
const FrameMode MODE_HSV = asn1Sccmode_HSV;
const FrameMode MODE_HLS = asn1Sccmode_HLS;
const FrameMode MODE_YUV = asn1Sccmode_YUV;
const FrameMode MODE_UYVY = asn1Sccmode_UYVY;
const FrameMode MODE_LAB = asn1Sccmode_Lab;
const FrameMode MODE_LUV = asn1Sccmode_Luv;
const FrameMode MODE_XYZ = asn1Sccmode_XYZ;
const FrameMode MODE_YCRCB = asn1Sccmode_YCrCb;
const FrameMode MODE_RGB32 = asn1Sccmode_RGB32;
const FrameMode MODE_BAYER_RGGB = asn1Sccmode_Bayer_RGGB;
const FrameMode MODE_BAYER_GRBG = asn1Sccmode_Bayer_GRBG;
const FrameMode MODE_BAYER_BGGR = asn1Sccmode_Bayer_BGGR;
const FrameMode MODE_BAYER_GBRG = asn1Sccmode_Bayer_GBRG;
const FrameMode MODE_PJPG = asn1Sccmode_PJPG;
const FrameMode MODE_JPEG = asn1Sccmode_JPEG;
const FrameMode MODE_PNG = asn1Sccmode_PNG;

const FrameStatus STATUS_EMPTY = asn1Sccstatus_EMPTY;
const FrameStatus STATUS_VALID = asn1Sccstatus_VALID;
const FrameStatus STATUS_INVALID = asn1Sccstatus_INVALID;

// Global constant variables

const int MAX_FRAME_ATTRIBUTES = frameMaxAttributes;
const int MAX_FRAME_ERROR_VALUES = frameMaxErrValues;
const int FRAME_VERSION = frame_Version;

// Pointer types

typedef Frame* FramePtr;
typedef Frame const* FrameConstPtr;
typedef std::shared_ptr<Frame> FrameSharedPtr;
typedef std::shared_ptr<const Frame> FrameSharedConstPtr;

// Functions

FramePtr NewFrame();
FrameSharedPtr NewSharedFrame();
FrameConstPtr Clone(const Frame& source);
FrameSharedPtr SharedClone(const Frame& source);

/**
 * Initialize the fields of the frame to sane default values.
 * @param frame The frame to initialize
 */
void Initialize(Frame& frame);

/**
 * Copy the data and the attributes of the source frame to the destination. This
 * does not invalidate the source frame.
 *
 * @param source The frame from which to copy the data and attributes
 * @param destination The frame into which to copy the data and attributes
 */
void Copy(const Frame& source, Frame& destination);

void SetFrameMode(Frame& frame, FrameMode frameMode);
FrameMode GetFrameMode(const Frame& frame);

void SetFrameSize(Frame& frame, BaseTypesWrapper::T_UInt16 width, BaseTypesWrapper::T_UInt16 height);
BaseTypesWrapper::T_UInt16 GetFrameWidth(const Frame& frame);
BaseTypesWrapper::T_UInt16 GetFrameHeight(const Frame& frame);

void SetFrameTime(Frame& frame, BaseTypesWrapper::T_Int64 time);
BaseTypesWrapper::T_UInt64 GetFrameTime(const Frame& frame);

void SetFrameReceivedTime(Frame& frame, BaseTypesWrapper::T_Int64 time);
BaseTypesWrapper::T_UInt64 GetFrameReceivedTime(const Frame& frame);

void SetFrameStatus(Frame& frame, FrameStatus frameStatus);
FrameStatus GetFrameStatus(const Frame& frame);

void ClearData(Frame& frame, bool overwrite = false);
byte GetDataByte(const Frame& frame, int index);
int GetNumberOfDataBytes(const Frame& frame);

BitStream ConvertToBitStream(const Frame& frame);
void ConvertFromBitStream(BitStream bitStream, Frame& frame);

    /* !! ASSUMES Little Endian */
    template<typename T>
    void AppendData(Frame &frame, T data)
    {
        ASSERT_ON_TEST(frame.data.data.nCount + static_cast<int>(sizeof(T)) < Array3DWrapper::MAX_ARRAY3D_BYTE_SIZE, "Image data will exceed limits");
        std::memcpy(&frame.data.data.arr[frame.data.data.nCount], &data, sizeof (T));
        frame.data.data.nCount+= sizeof (T);
    }
}

#endif // FRAME_HPP

/** @} */
