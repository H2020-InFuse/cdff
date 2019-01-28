/**
 * @addtogroup DFNs
 * @{
 */

#include "ImageDegradationInterface.hpp"

namespace CDFF
{
namespace DFN
{

ImageDegradationInterface::ImageDegradationInterface()
{
    asn1SccFrame_Initialize(& inOriginalImage);
    asn1SccFrame_Initialize(& outDegradedImage);
}

ImageDegradationInterface::~ImageDegradationInterface()
{
}

void ImageDegradationInterface::originalImageInput(const asn1SccFrame& data)
{
    inOriginalImage = data;
}

const asn1SccFrame& ImageDegradationInterface::degradedImageOutput() const
{
    return outDegradedImage;
}

}
}

/** @} */
