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
