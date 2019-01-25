/**
 * @addtogroup DFNs
 * @{
 */

#include "ImageFilteringInterface.hpp"

namespace CDFF
{
namespace DFN
{

ImageFilteringInterface::ImageFilteringInterface()
{
    asn1SccFrame_Initialize(& inImage);
    asn1SccFrame_Initialize(& outImage);
}

ImageFilteringInterface::~ImageFilteringInterface()
{
}

void ImageFilteringInterface::imageInput(const asn1SccFrame& data)
{
    inImage = data;
}

const asn1SccFrame& ImageFilteringInterface::imageOutput() const
{
    return outImage;
}

}
}

/** @} */
