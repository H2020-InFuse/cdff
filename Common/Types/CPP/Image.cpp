/**
 * @author Alessandro Bianco, Vincent Bissonnette
 */

/**
 * @addtogroup ImageWrapper
 * @{
 */

#include "Image.hpp"
#include <Errors/Assert.hpp>

namespace ImageWrapper
{

using namespace BaseTypesWrapper;

void Copy(const Image& source, Image& destination)
{
    SetImageTime(destination, GetImageTime(source));
	SetReceivedTime(destination, GetReceivedTime(source));
	SetDataDepth(destination, GetDataDepth(source));
	SetPixelSize(destination, GetPixelSize(source));
	SetRowSize(destination, GetRowSize(source));
    SetImageMode(destination, GetImageMode(source));
    SetImageStatus(destination, GetImageStatus(source));
    SetImageSize(destination, GetImageSize(source));
	ClearAttributes(destination);
	for (unsigned attributeIndex = 0; attributeIndex < GetNumberOfAttributes(source); attributeIndex++)
	{
		AddAttribute(destination, GetAttribute(source, attributeIndex));
	}
	ClearData(destination);
	for (int dataByteIndex = 0; dataByteIndex < GetNumberOfDataBytes(source); dataByteIndex++)
	{
		AddDataByte(destination, GetDataByte(source, dataByteIndex));
	}
}

ImagePtr NewImage()
{
    ImagePtr frame = new Image();
	Initialize(*frame);
	return frame;
}

ImageSharedPtr NewSharedImage()
{
    ImageSharedPtr sharedFrame = std::make_shared<Image>();
	Initialize(*sharedFrame);
	return sharedFrame;
}

ImageConstPtr Clone(const Image& source)
{
    ImagePtr frame = new Image();
	Copy(source, *frame);
	return frame;
}

ImageSharedPtr SharedClone(const Image& source)
{
    ImageSharedPtr sharedFrame = std::make_shared<Image>();
	Copy(source, *sharedFrame);
	return sharedFrame;
}

void Initialize(Image& frame)
{
    SetImageTime(frame, 0);
    SetReceivedTime(frame, 0);
    SetDataDepth(frame, 0);
    SetPixelSize(frame, 0);
    SetRowSize(frame, 0);
    SetImageSize(frame, 0, 0);
    SetImageMode(frame, MODE_UNDEFINED);
    SetImageStatus(frame, STATUS_EMPTY);
    ClearAttributes(frame);
    ClearData(frame);
}

void SetImageTime(Image& image, T_Int64 time)
{
    image.frame_time.microseconds = time;
    image.frame_time.usecPerSec = 1;
}

T_Int64 GetImageTime(const Image& frame)
{
    return frame.frame_time.microseconds;
}

void SetReceivedTime(Image& frame, T_Int64 time)
{
    frame.received_time.microseconds = time;
    frame.received_time.usecPerSec = 1;
}

T_Int64 GetReceivedTime(const Image& frame)
{
    return frame.received_time.microseconds;
}

void SetDataDepth(Image& frame, T_UInt32 dataDepth)
{
    frame.data_depth = dataDepth;
}

T_UInt32 GetDataDepth(const Image& frame)
{
    return frame.data_depth;
}

void SetPixelSize(Image& frame, T_UInt32 pixelSize)
{
    frame.pixel_size = pixelSize;
}

T_UInt32 GetPixelSize(const Image& frame)
{
    return frame.pixel_size;
}

void SetRowSize(Image& frame, T_UInt32 rowSize)
{
	frame.row_size = rowSize;
}

T_UInt32 GetRowSize(const Image& frame)
{
	return frame.row_size;
}

void SetImageMode(Image& frame, ImageMode frameMode)
{
	frame.frame_mode = frameMode;
}

ImageMode GetImageMode(const Image& frame)
{
	return frame.frame_mode;
}

void SetImageStatus(Image& frame, ImageStatus frameStatus)
{
	frame.frame_status = frameStatus;
}

ImageStatus GetImageStatus(const Image& frame)
{
	return frame.frame_status;
}

void SetImageSize(Image& frame, T_UInt16 width, T_UInt16 height)
{
	frame.datasize.width = width;
	frame.datasize.height = height;
}

void SetImageSize(Image& frame, ImageSize imageSize)
{
    frame.datasize.width = imageSize.width;
    frame.datasize.height = imageSize.height;
}

T_UInt16 GetImageWidth(const Image& frame)
{
	return frame.datasize.width;
}

T_UInt16 GetImageHeight(const Image& frame)
{
	return frame.datasize.height;
}

ImageSize GetImageSize(const Image& frame)
{
	return frame.datasize;
}

void AddAttribute(Image& frame, T_String data, T_String name)
{
    ASSERT_ON_TEST(frame.attributes.nCount < MAX_IMAGE_ATTRIBUTES, "Adding more Frame attributes than allowed");
	int currentIndex = frame.attributes.nCount;
	frame.attributes.arr[currentIndex].data = data;
	frame.attributes.arr[currentIndex].att_name = name;
	frame.attributes.nCount++;
}

void AddAttribute(Image& frame, ImageAttribute attribute)
{
    ASSERT_ON_TEST(frame.attributes.nCount < MAX_IMAGE_ATTRIBUTES, "Adding more Frame attributes than allowed");
	int currentIndex = frame.attributes.nCount;
	frame.attributes.arr[currentIndex].data = attribute.data;
	frame.attributes.arr[currentIndex].att_name = attribute.att_name;
	frame.attributes.nCount++;
}

void ClearAttributes(Image& frame)
{
	frame.attributes.nCount = 0;
}

void RemoveAttribute(Image& frame, int index)
{
	ASSERT_ON_TEST(index < frame.attributes.nCount, "Requesting a missing attribute from a Frame");
	for (int iteratorIndex = index; iteratorIndex <  frame.attributes.nCount - 1; iteratorIndex++)
	{
		frame.attributes.arr[iteratorIndex] = frame.attributes.arr[iteratorIndex+1];
	}
	frame.attributes.nCount--;
}

ImageAttribute GetAttribute(const Image& frame, int index)
{
	ASSERT_ON_TEST(index < frame.attributes.nCount, "Requesting a missing attribute from a Frame");
	return frame.attributes.arr[index];
}

unsigned GetNumberOfAttributes(const Image& frame)
{
	return frame.attributes.nCount;
}

void AddDataByte(Image& frame, byte data)
{
	ASSERT_ON_TEST(frame.image.nCount < MAX_DATA_BYTE_SIZE, "Image data exceeds limits");
	int currentIndex = frame.image.nCount;
	frame.image.arr[currentIndex] = data;
	frame.image.nCount++;
}

void ClearData(Image& frame)
{
	frame.image.nCount = 0;
}

byte GetDataByte(const Image& frame, int index)
{
	ASSERT_ON_TEST(index < frame.image.nCount, "Requesting missing image data");
	return frame.image.arr[index];
}

int GetNumberOfDataBytes(const Image& frame)
{
	return frame.image.nCount;
}

BitStream ConvertToBitStream(const Image& frame)
    CONVERT_TO_BIT_STREAM(frame, asn1SccImage_REQUIRED_BYTES_FOR_ENCODING, asn1SccImage_Encode)

void ConvertFromBitStream(BitStream bitStream, Image& frame)
    CONVERT_FROM_BIT_STREAM(bitStream, asn1SccImage_REQUIRED_BYTES_FOR_ENCODING, frame, asn1SccImage_Decode)

}

/** @} */
