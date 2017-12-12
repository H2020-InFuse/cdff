/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Frame.cpp
 * @date 05/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CppTypes
 * 
 * Implementation of Frame class.
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
#include "Frame.hpp"
#include <Errors/Assert.hpp>

namespace CppTypes
{


/* --------------------------------------------------------------------------
 *
 * Public Member Variables
 *
 * --------------------------------------------------------------------------
 */
const FrameMode Frame::MODE_UNDEFINED = CTypes::mode_undefined;
const FrameMode Frame::MODE_GRAYSCALE = CTypes::mode_grayscale;
const FrameMode Frame::MODE_RGB = CTypes::mode_rgb;
const FrameMode Frame::MODE_UYVY = CTypes::mode_uyvy;
const FrameMode Frame::MODE_BGR = CTypes::mode_bgr;
const FrameMode Frame::MODE_RGB32 = CTypes::mode_rgb32;
const FrameMode Frame::RAW_MODES = CTypes::raw_modes;
const FrameMode Frame::MODE_BAYER = CTypes::mode_bayer;
const FrameMode Frame::MODE_BAYER_RGGB = CTypes::mode_bayer_rggb;
const FrameMode Frame::MODE_BAYER_GRBG = CTypes::mode_bayer_grbg;
const FrameMode Frame::MODE_BAYER_BGGR = CTypes::mode_bayer_bggr;
const FrameMode Frame::MODE_BAYER_GBRG = CTypes::mode_bayer_gbrg;
const FrameMode Frame::COMPRESSED_MODES = CTypes::compressed_modes;
const FrameMode Frame::MODE_PJPG = CTypes::Frame_mode_t_mode_pjpg;
const FrameMode Frame::MODE_JPEG = CTypes::mode_jpeg;
const FrameMode Frame::MODE_PNG = CTypes::mode_png;

const FrameStatus Frame::STATUS_EMPTY = CTypes::status_empty;
const FrameStatus Frame::STATUS_VALID = CTypes::status_valid;
const FrameStatus Frame::STATUS_INVALID = CTypes::status_invalid;


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
Frame::Frame()
	{
	ClearAttributes();
	ClearData();
	}

Frame::~Frame()
	{

	}

void Frame::SetFrameTime(T_Int64 time)
	{
	frame.frame_time.microseconds = time;
	frame.frame_time.usecPerSec = 1;
	}

T_Int64 Frame::GetFrameTime() const
	{
	return frame.frame_time.microseconds;
	}

void Frame::SetReceivedTime(T_Int64 time)
	{
	frame.received_time.microseconds = time;
	frame.received_time.usecPerSec = 1;
	}

T_Int64 Frame::GetReceivedTime() const
	{
	return frame.received_time.microseconds;
	}

void Frame::SetDataDepth(T_UInt32 dataDepth)
	{
	frame.data_depth = dataDepth;
	}

T_UInt32 Frame::GetDataDepth() const
	{
	return frame.data_depth;
	}

void Frame::SetPixelSize( T_UInt32 pixelSize)
	{
	frame.pixel_size = pixelSize;
	}

T_UInt32 Frame::GetPixelSize() const
	{
	return frame.pixel_size;
	}

void Frame::SetRowSize(T_UInt32 rowSize)
	{
	frame.row_size = rowSize;
	}

T_UInt32 Frame::GetRowSize() const
	{
	return frame.row_size;
	}

void Frame::SetFrameMode(FrameMode frameMode)
	{
	frame.frame_mode = frameMode;
	}

FrameMode Frame::GetFrameMode() const
	{
	return frame.frame_mode;
	}
		
void Frame::SetFrameStatus(FrameStatus frameStatus)
	{
	frame.frame_status = frameStatus;
	}

FrameStatus Frame::GetFrameStatus() const
	{
	return frame.frame_status;
	}

void Frame::SetFrameSize(T_UInt16 width, T_UInt16 height)
	{
	frame.datasize.width = width;
	frame.datasize.height = height;
	}

void Frame::SetFrameSize(FrameSize frameSize)
	{
	frame.datasize.width = frameSize.width;
	frame.datasize.height = frameSize.height;
	}

T_UInt16 Frame::GetFrameWidth() const
	{
	return frame.datasize.width;
	}

T_UInt16 Frame::GetFrameHeight() const
	{
	return frame.datasize.height;
	}

FrameSize Frame::GetFrameSize() const
	{
	return frame.datasize;
	}
		
void Frame::AddAttribute(T_String data, T_String name)
	{
	ASSERT_ON_TEST(frame.attributes.nCount < MAX_FRAME_ATTRIBUTES, "Adding more Frame attributes than allowed");
	int currentIndex = frame.attributes.nCount;	
	frame.attributes.arr[currentIndex].data = data;
	frame.attributes.arr[currentIndex].att_name = name;
	frame.attributes.nCount++;
	}

void Frame::ClearAttributes()
	{
	frame.attributes.nCount = 0;
	}

void Frame::RemoveAttribute(int index)
	{
	ASSERT_ON_TEST(index < frame.attributes.nCount, "Requesting a missing attribute from a Frame");
	for(int iteratorIndex = index; iteratorIndex <  frame.attributes.nCount - 1; iteratorIndex++)
		{
		frame.attributes.arr[iteratorIndex] = frame.attributes.arr[iteratorIndex+1];
		} 
	frame.attributes.nCount--;
	}

FrameAttribute Frame::GetAttribute(int index) const
	{
	ASSERT_ON_TEST(index < frame.attributes.nCount, "Requesting a missing attribute from a Frame");
	return frame.attributes.arr[index];
	}

unsigned Frame::GetNumberOfAttributes() const
	{
	return frame.attributes.nCount;
	}
	
void Frame::AddDataByte(byte data)
	{
	ASSERT_ON_TEST(frame.image.nCount < MAX_DATA_BYTE_SIZE, "Image data exceeds limits");
	int currentIndex = frame.image.nCount;
	frame.image.arr[currentIndex] = data;
	frame.image.nCount++;
	}

void Frame::ClearData()
	{
	frame.image.nCount = 0;
	}

byte Frame::GetDataByte(int index) const
	{
	ASSERT_ON_TEST(index < frame.image.nCount, "Requesting missing image data");
	return frame.image.arr[index];
	}

int Frame::GetNumberOfDataBytes() const
	{
	return frame.image.nCount;
	}


/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const int Frame::MAX_FRAME_ATTRIBUTES = CTypes::frameMaxAttributes;
const int Frame::MAX_DATA_BYTE_SIZE = CTypes::frameMaxBytes;


}

/** @} */
