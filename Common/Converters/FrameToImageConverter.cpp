/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FrameToImageConverter.cpp
 * @date 02/08/2018
 * @author Vincent Bissonnette
 */

/*!
 * @addtogroup Converters
 *
 * Implementation of FrameToImageConverter.
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

#include "FrameToImageConverter.hpp"
#include <Errors/Assert.hpp>
#include<iostream>

namespace Converters {

using namespace FrameWrapper;
using namespace ImageWrapper;
using namespace Array3DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

const ImageWrapper::ImageConstPtr FrameToImageConverter::Convert(const FrameWrapper::FrameConstPtr& frame)
{
    ImagePtr image = new Image();
    if (GetFrameHeight(*frame) == 0 && GetFrameWidth(*frame) == 0)
    {
        return image;
    }

    SetImageTime(*image, GetFrameTime(*frame));
    SetReceivedTime(*image, GetFrameReceivedTime(*frame));

    Array3DDepth frameDepth = GetArray3DDepth(frame->data);
    BaseTypesWrapper::T_UInt32 depth_bytes = 0;
    if((frameDepth == ARRAY3D_8U) || (frameDepth == ARRAY3D_8S))
    {
        depth_bytes = 1;
    }
    else if((frameDepth == ARRAY3D_16U) || (frameDepth == ARRAY3D_16S))
    {
        depth_bytes = 2;
    }
    else if((frameDepth == ARRAY3D_32F) || (frameDepth == ARRAY3D_32S))
    {
        depth_bytes = 4;
    }
    else if((frameDepth == ARRAY3D_64F))
    {
        depth_bytes = 8;
    }
    else
    {
        depth_bytes = 0;
        ASSERT(false, "Unhandled frame depth in FrameToImageConverter");
    }

    SetDataDepth(*image, depth_bytes);
    SetPixelSize(*image, depth_bytes * frame->data.channels);
    SetRowSize(*image, Array3DWrapper::GetArray3DRowSize(frame->data));
    SetImageMode(*image, ConvertFrameModeToImageMode(GetFrameMode(*frame)));
    SetImageSize(*image, frame->data.cols, frame->data.rows);

    if(GetFrameStatus(*frame) == FrameWrapper::STATUS_EMPTY)
    {
        SetImageStatus(*image, ImageWrapper::STATUS_EMPTY);
    }
    else if(GetFrameStatus(*frame) == FrameWrapper::STATUS_INVALID)
    {
        SetImageStatus(*image, ImageWrapper::STATUS_INVALID);
    }
    else if(GetFrameStatus(*frame) == FrameWrapper::STATUS_VALID)
    {
        SetImageStatus(*image, ImageWrapper::STATUS_VALID);
    }

    image->attributes.nCount = 0;
    for(int attIndex = 0; attIndex < (frame->metadata.attributes.nCount); ++attIndex)
    {
        AddAttribute(*image, frame->metadata.attributes.arr[attIndex].data, frame->metadata.attributes.arr[attIndex].name);
    }

    // copy buffers
    ASSERT_ON_TEST(frame->data.data.nCount < ImageWrapper::MAX_DATA_BYTE_SIZE, "Frame data exceeds Image size limits");
    std::memcpy(&image->image.arr, &frame->data.data.arr, frame->data.data.nCount);
    image->image.nCount = frame->data.data.nCount;

    return image;
}


const ImageWrapper::ImageSharedConstPtr FrameToImageConverter::ConvertShared(const FrameWrapper::FrameSharedConstPtr& frame)
{
    ImageConstPtr image = Convert(frame.get());
    ImageSharedConstPtr sharedImage(image);
    return sharedImage;
}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
const ImageWrapper::ImageMode FrameToImageConverter::ConvertFrameModeToImageMode(const FrameWrapper::FrameMode& frameMode)
{
    ImageMode imageMode = ImageWrapper::MODE_UNDEFINED;

    if(frameMode == FrameWrapper::MODE_UNDEFINED)
    {
        imageMode = ImageWrapper::MODE_UNDEFINED;
    }
    else if(frameMode == FrameWrapper::MODE_GRAYSCALE)
    {
        imageMode = ImageWrapper::MODE_GRAYSCALE;
    }
    else if(frameMode == FrameWrapper::MODE_RGB)
    {
        imageMode = ImageWrapper::MODE_RGB;
    }
    else if(frameMode == FrameWrapper::MODE_RGBA)
    {
        imageMode = ImageWrapper::MODE_UNDEFINED;
    }
    else if(frameMode == FrameWrapper::MODE_BGR)
    {
        imageMode = ImageWrapper::MODE_BGR;
    }
    else if(frameMode == FrameWrapper::MODE_BGRA)
    {
        imageMode = ImageWrapper::MODE_UNDEFINED;
    }
    else if(frameMode == FrameWrapper::MODE_HSV)
    {
        imageMode = ImageWrapper::MODE_UNDEFINED;
    }
    else if(frameMode == FrameWrapper::MODE_HLS)
    {
        imageMode = ImageWrapper::MODE_UNDEFINED;
    }
    else if(frameMode == FrameWrapper::MODE_YUV)
    {
        imageMode = ImageWrapper::MODE_UNDEFINED;
    }
    else if(frameMode == FrameWrapper::MODE_UYVY)
    {
        imageMode = ImageWrapper::MODE_UYVY;
    }
    else if(frameMode == FrameWrapper::MODE_LAB)
    {
        imageMode = ImageWrapper::MODE_UNDEFINED;
    }
    else if(frameMode == FrameWrapper::MODE_LUV)
    {
        imageMode = ImageWrapper::MODE_UNDEFINED;
    }
    else if(frameMode == FrameWrapper::MODE_XYZ)
    {
        imageMode = ImageWrapper::MODE_UNDEFINED;
    }
    else if(frameMode == FrameWrapper::MODE_YCRCB)
    {
        imageMode = ImageWrapper::MODE_UNDEFINED;
    }
    else if(frameMode == FrameWrapper::MODE_RGB32)
    {
        imageMode = ImageWrapper::MODE_RGB32;
    }
    else if(frameMode == FrameWrapper::MODE_BAYER_RGGB)
    {
        imageMode = ImageWrapper::MODE_BAYER_RGGB;
    }
    else if(frameMode == FrameWrapper::MODE_BAYER_GRBG)
    {
        imageMode = ImageWrapper::MODE_BAYER_GRBG;
    }
    else if(frameMode == FrameWrapper::MODE_BAYER_BGGR)
    {
        imageMode = ImageWrapper::MODE_BAYER_BGGR;
    }
    else if(frameMode == FrameWrapper::MODE_BAYER_GBRG)
    {
        imageMode = ImageWrapper::MODE_BAYER_GBRG;
    }
    else if(frameMode == FrameWrapper::MODE_PJPG)
    {
        imageMode = ImageWrapper::MODE_PJPG;
    }
    else if(frameMode == FrameWrapper::MODE_JPEG)
    {
        imageMode = ImageWrapper::MODE_JPEG;
    }
    else if(frameMode == FrameWrapper::MODE_PNG)
    {
        imageMode = ImageWrapper::MODE_PNG;
    }
    else {
        imageMode = ImageWrapper::MODE_UNDEFINED;
    }
    return imageMode;
}

}

/** @} */
