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
