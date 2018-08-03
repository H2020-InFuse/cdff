/**
 * @author Alessandro Bianco, Vincent Bissonnette
 */

/**
 * @addtogroup ImageWrapper
 *
 * Wrapper for ASN.1 Image type
 * Note: This type was previously called Frame.
 * It has been renamed to Image to make space for the updated Frame type.
 * @{
 */

#ifndef IMAGE_HPP
#define IMAGE_HPP

#include <Image.h>

#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>

namespace ImageWrapper
{

// Types

typedef asn1SccImage_mode_t ImageMode;
typedef asn1SccImage_status_t ImageStatus;
typedef asn1SccImage_size_t ImageSize;
typedef asn1SccImage_attrib_t ImageAttribute;
typedef asn1SccImage_image ImageImage;
typedef asn1SccImage_attributes ImageAttributesList;
typedef asn1SccImage Image;
typedef asn1SccImagePair ImagePair;

// Enumerated types

const ImageMode MODE_UNDEFINED = asn1Sccmode_undefined;
const ImageMode MODE_GRAYSCALE = asn1Sccmode_grayscale;
const ImageMode MODE_RGB = asn1Sccmode_rgb;
const ImageMode MODE_UYVY = asn1Sccmode_uyvy;
const ImageMode MODE_BGR = asn1Sccmode_bgr;
const ImageMode MODE_RGB32 = asn1Sccmode_rgb32;
const ImageMode RAW_MODES = asn1Sccraw_modes;
const ImageMode MODE_BAYER = asn1Sccmode_bayer;
const ImageMode MODE_BAYER_RGGB = asn1Sccmode_bayer_rggb;
const ImageMode MODE_BAYER_GRBG = asn1Sccmode_bayer_grbg;
const ImageMode MODE_BAYER_BGGR = asn1Sccmode_bayer_bggr;
const ImageMode MODE_BAYER_GBRG = asn1Sccmode_bayer_gbrg;
const ImageMode COMPRESSED_MODES = asn1Scccompressed_modes;
const ImageMode MODE_PJPG = asn1SccImage_mode_t_mode_pjpg;
const ImageMode MODE_JPEG = asn1Sccmode_jpeg;
const ImageMode MODE_PNG = asn1Sccmode_png;

const ImageStatus STATUS_EMPTY = asn1Sccstatus_empty;
const ImageStatus STATUS_VALID = asn1Sccstatus_valid;
const ImageStatus STATUS_INVALID = asn1Sccstatus_invalid;

// Global constant variables

const int MAX_IMAGE_ATTRIBUTES = imageMaxAttributes;
const int MAX_DATA_BYTE_SIZE = imageMaxBytes;

// Pointer types

typedef Image* ImagePtr;
typedef Image const* ImageConstPtr;
typedef std::shared_ptr<Image> ImageSharedPtr;
typedef std::shared_ptr<const Image> ImageSharedConstPtr;

// Functions

void Copy(const Image& source, Image& destination);
ImagePtr NewImage();
ImageSharedPtr NewSharedImage();
ImageConstPtr Clone(const Image& source);
ImageSharedPtr SharedClone(const Image& source);
void Initialize(Image& frame);

void SetImageTime(Image& image, BaseTypesWrapper::T_Int64 time);
BaseTypesWrapper::T_Int64 GetImageTime(const Image& frame);

void SetReceivedTime(Image& frame, BaseTypesWrapper::T_Int64 time);
BaseTypesWrapper::T_Int64 GetReceivedTime(const Image& frame);

/**
 * @brief SetDataDepth TBC : Number of bytes of 1 pixel, on 1 channel
 * @param frame Image on which to set property
 * @param dataDepth Number of bytes of 1 pixel, on 1 channel.
 */
void SetDataDepth(Image& frame, BaseTypesWrapper::T_UInt32 dataDepth);
BaseTypesWrapper::T_UInt32 GetDataDepth(const Image& frame);

/**
 * @brief SetPixelSize Total number of bytes of 1 pixel, with all its channels.
 * @param frame Image on which to set property
 * @param pizelSize Total number of bytes of 1 pixel, with all its channels.
 */
void SetPixelSize(Image& frame, BaseTypesWrapper::T_UInt32 pixelSize);
BaseTypesWrapper::T_UInt32 GetPixelSize(const Image& frame);

void SetRowSize(Image& frame, BaseTypesWrapper::T_UInt32 rowSize);
BaseTypesWrapper::T_UInt32 GetRowSize(const Image& frame);

void SetImageMode(Image& frame, ImageMode frameMode);
ImageMode GetImageMode(const Image& frame);

void SetImageStatus(Image& frame, ImageStatus frameStatus);
ImageStatus GetImageStatus(const Image& frame);

void SetImageSize(Image& frame, BaseTypesWrapper::T_UInt16 width, BaseTypesWrapper::T_UInt16 height);
void SetImageSize(Image& frame, ImageSize imageSize);
BaseTypesWrapper::T_UInt16 GetImageWidth(const Image& frame);
BaseTypesWrapper::T_UInt16 GetImageHeight(const Image& frame);
ImageSize GetImageSize(const Image& frame);

void AddAttribute(Image& frame, BaseTypesWrapper::T_String data, BaseTypesWrapper::T_String name);
void AddAttribute(Image& frame, ImageAttribute attribute);
void ClearAttributes(Image& frame);
void RemoveAttribute(Image& frame, int index);
ImageAttribute GetAttribute(const Image& frame, int index);
unsigned GetNumberOfAttributes(const Image& frame);

void AddDataByte(Image& frame, byte data);
void ClearData(Image& frame);
byte GetDataByte(const Image& frame, int index);
int GetNumberOfDataBytes(const Image& frame);

BitStream ConvertToBitStream(const Image& frame);
void ConvertFromBitStream(BitStream bitStream, Image& frame);

}

#endif // FRAME_HPP

/** @} */
