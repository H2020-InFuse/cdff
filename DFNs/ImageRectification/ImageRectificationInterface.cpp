/**
 * @addtogroup DFNs
 * @{
 */

#include "ImageRectificationInterface.hpp"

namespace CDFF
{
namespace DFN
{

ImageRectificationInterface::ImageRectificationInterface()
{
    asn1SccFrame_Initialize(& inOriginalImage);
    asn1SccFrame_Initialize(& outRectifiedImage);
}

ImageRectificationInterface::~ImageRectificationInterface()
{
}

void ImageRectificationInterface::originalImageInput(const asn1SccFrame& data)
{
    inOriginalImage = data;
}

const asn1SccFrame& ImageRectificationInterface::rectifiedImageOutput() const
{
    return outRectifiedImage;
}

}
}

/** @} */
