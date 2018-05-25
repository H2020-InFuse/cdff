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

namespace FrameWrapper
{

// Types

typedef asn1SccFrame_mode_t FrameMode;
typedef asn1SccFrame_status_t FrameStatus;
typedef asn1SccFrame_size_t FrameSize;
typedef asn1SccFrame_attrib_t FrameAttribute;
typedef asn1SccFrame_image FrameImage;
typedef asn1SccFrame_attributes FrameAttributesList;
typedef asn1SccFrame Frame;
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

void Copy(const Frame& source, Frame& destination);
FramePtr NewFrame();
FrameSharedPtr NewSharedFrame();
FrameConstPtr Clone(const Frame& source);
FrameSharedPtr SharedClone(const Frame& source);
void Initialize(Frame& frame);

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
