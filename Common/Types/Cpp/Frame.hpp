/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Frame.hpp
 * @date 12/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup FrameWrapper
 * 
 * Frame namespace wrapper for Frame type
 * 
 * 
 * @{
 */


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
namespace CTypes {
#include <Frame.h>
}
#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>

#ifndef FRAME_HPP
#define FRAME_HPP


/* --------------------------------------------------------------------------
 *
 * Frame namespace
 *
 * --------------------------------------------------------------------------
 */
namespace FrameWrapper
{

/* --------------------------------------------------------------------------
 *
 * Types definition
 *
 * --------------------------------------------------------------------------
 */
typedef CTypes::Frame_mode_t FrameMode;
typedef CTypes::Frame_status_t FrameStatus;
typedef CTypes::Frame_size_t FrameSize;
typedef CTypes::Frame_attrib_t FrameAttribute;
typedef CTypes::Frame_image FrameImage;
typedef CTypes::Frame_attributes FrameAttributesList;
typedef CTypes::Frame Frame;
typedef CTypes::FramePair FramePair;


/* --------------------------------------------------------------------------
 *
 * Constants definition
 *
 * --------------------------------------------------------------------------
 */
const FrameMode MODE_UNDEFINED = CTypes::mode_undefined;
const FrameMode MODE_GRAYSCALE = CTypes::mode_grayscale;
const FrameMode MODE_RGB = CTypes::mode_rgb;
const FrameMode MODE_UYVY = CTypes::mode_uyvy;
const FrameMode MODE_BGR = CTypes::mode_bgr;
const FrameMode MODE_RGB32 = CTypes::mode_rgb32;
const FrameMode RAW_MODES = CTypes::raw_modes;
const FrameMode MODE_BAYER = CTypes::mode_bayer;
const FrameMode MODE_BAYER_RGGB = CTypes::mode_bayer_rggb;
const FrameMode MODE_BAYER_GRBG = CTypes::mode_bayer_grbg;
const FrameMode MODE_BAYER_BGGR = CTypes::mode_bayer_bggr;
const FrameMode MODE_BAYER_GBRG = CTypes::mode_bayer_gbrg;
const FrameMode COMPRESSED_MODES = CTypes::compressed_modes;
const FrameMode MODE_PJPG = CTypes::Frame_mode_t_mode_pjpg;
const FrameMode MODE_JPEG = CTypes::mode_jpeg;
const FrameMode MODE_PNG = CTypes::mode_png;

const FrameStatus STATUS_EMPTY = CTypes::status_empty;
const FrameStatus STATUS_VALID = CTypes::status_valid;
const FrameStatus STATUS_INVALID = CTypes::status_invalid;

const int MAX_FRAME_ATTRIBUTES = CTypes::frameMaxAttributes;
const int MAX_DATA_BYTE_SIZE = CTypes::frameMaxBytes;


/* --------------------------------------------------------------------------
 *
 * Shared Pointers definition
 *
 * --------------------------------------------------------------------------
 */
typedef std::shared_ptr<Frame> FrameSharedPtr;
typedef std::shared_ptr<const Frame> FrameSharedConstPtr;
typedef Frame* FramePtr;
typedef Frame const* FrameConstPtr;



/* --------------------------------------------------------------------------
 *
 * Access Functions definition
 *
 * --------------------------------------------------------------------------
 */
void Copy(const Frame& source, Frame& destination);
FramePtr NewFrame();
FrameSharedPtr NewSharedFrame();
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
	
void AddDataByte(Frame& frame, BaseTypesWrapper::byte data);
void ClearData(Frame& frame);
BaseTypesWrapper::byte GetDataByte(const Frame& frame, int index);
int GetNumberOfDataBytes(const Frame& frame);

}

#endif

/* Frame.h */
/** @} */
