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

#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>

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
 *     `number_of_channels * bytes_per_channel`,
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

/// The `FrameMode` documents the pixel format used for the frame data. The
// frame modes can indicate the packing of the colour components of each pixel
/// (RGB, BGR, Grayscale, ...) or the type of compression used on the data
// (PNG, JPEG, ...).
typedef asn1SccFrame_mode_t FrameMode;

/// The `FrameStatus` indicates whether the data buffer contains valid data or not.
typedef asn1SccFrame_status_t FrameStatus;

/// The `FrameSize` contains the size of the image in rows and columns.
typedef asn1SccFrame_size_t FrameSize;

/// ???
typedef asn1SccFrame_attrib_t FrameAttribute;

/// The actual image data. The `FrameImage` is a struct composed of a fixed
/// size buffer of byte (`arr`) and a size member (`nCount`) which indicates
/// the number of bytes used to store the image.
typedef asn1SccFrame_image FrameImage;

typedef asn1SccFrame_attributes FrameAttributesList;

/// Container type for the raw image data and metadata
typedef asn1SccFrame Frame;

// Container types for synchronised frame pairs (e.g. stereo pairs)
typedef asn1SccFramePair FramePair;

// Enumerated types

const FrameMode MODE_UNDEFINED = asn1Sccmode_undefined;
const FrameMode MODE_GRAYSCALE = asn1Sccmode_grayscale;
const FrameMode MODE_RGB = asn1Sccmode_rgb;
const FrameMode MODE_UYVY = asn1Sccmode_uyvy;
const FrameMode MODE_BGR = asn1Sccmode_bgr;
const FrameMode MODE_RGB32 = asn1Sccmode_rgb32;
const FrameMode RAW_MODES = asn1Sccraw_modes;
const FrameMode MODE_BAYER = asn1Sccmode_bayer;
const FrameMode MODE_BAYER_RGGB = asn1Sccmode_bayer_rggb;
const FrameMode MODE_BAYER_GRBG = asn1Sccmode_bayer_grbg;
const FrameMode MODE_BAYER_BGGR = asn1Sccmode_bayer_bggr;
const FrameMode MODE_BAYER_GBRG = asn1Sccmode_bayer_gbrg;
const FrameMode COMPRESSED_MODES = asn1Scccompressed_modes;
const FrameMode MODE_PJPG = asn1SccFrame_mode_t_mode_pjpg;
const FrameMode MODE_JPEG = asn1Sccmode_jpeg;
const FrameMode MODE_PNG = asn1Sccmode_png;

const FrameStatus STATUS_EMPTY = asn1Sccstatus_empty;
const FrameStatus STATUS_VALID = asn1Sccstatus_valid;
const FrameStatus STATUS_INVALID = asn1Sccstatus_invalid;

// Global constant variables

const int MAX_FRAME_ATTRIBUTES = frameMaxAttributes;
const int MAX_DATA_BYTE_SIZE = frameMaxBytes;

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

void SetFrameTime(Frame& frame, BaseTypesWrapper::T_Int64 time);
BaseTypesWrapper::T_Int64 GetFrameTime(const Frame& frame);

void SetReceivedTime(Frame& frame, BaseTypesWrapper::T_Int64 time);
BaseTypesWrapper::T_Int64 GetReceivedTime(const Frame& frame);

void SetDataDepth(Frame& frame, BaseTypesWrapper::T_UInt32 dataDepth);
BaseTypesWrapper::T_UInt32 GetDataDepth(const Frame& frame);

void SetPixelSize(Frame& frame, BaseTypesWrapper::T_UInt32 pizelSize);
BaseTypesWrapper::T_UInt32 GetPixelSize(const Frame& frame);

void SetRowSize(Frame& frame, BaseTypesWrapper::T_UInt32 rowSize);
BaseTypesWrapper::T_UInt32 GetRowSize(const Frame& frame);

void SetFrameMode(Frame& frame, FrameMode frameMode);
FrameMode GetFrameMode(const Frame& frame);

void SetFrameStatus(Frame& frame, FrameStatus frameStatus);
FrameStatus GetFrameStatus(const Frame& frame);

void SetFrameSize(Frame& frame, BaseTypesWrapper::T_UInt16 width, BaseTypesWrapper::T_UInt16 height);
void SetFrameSize(Frame& frame, FrameSize frameSize);
BaseTypesWrapper::T_UInt16 GetFrameWidth(const Frame& frame);
BaseTypesWrapper::T_UInt16 GetFrameHeight(const Frame& frame);
FrameSize GetFrameSize(const Frame& frame);

void AddAttribute(Frame& frame, BaseTypesWrapper::T_String data, BaseTypesWrapper::T_String name);
void AddAttribute(Frame& frame, FrameAttribute attribute);
void ClearAttributes(Frame& frame);
void RemoveAttribute(Frame& frame, int index);
FrameAttribute GetAttribute(const Frame& frame, int index);
unsigned GetNumberOfAttributes(const Frame& frame);

void AddDataByte(Frame& frame, byte data);
void ClearData(Frame& frame);
byte GetDataByte(const Frame& frame, int index);
int GetNumberOfDataBytes(const Frame& frame);

}

#endif // FRAME_HPP

/** @} */
