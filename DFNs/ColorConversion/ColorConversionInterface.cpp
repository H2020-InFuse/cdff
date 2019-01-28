/**
 * @addtogroup DFNs
 * @{
 */

#include "ColorConversionInterface.hpp"

namespace CDFF
{
namespace DFN
{

ColorConversionInterface::ColorConversionInterface()
{
    asn1SccFrame_Initialize(&inOriginalImage);
    asn1SccFrame_Initialize(&outConvertedImage);
}

ColorConversionInterface::~ColorConversionInterface()
{
}

void ColorConversionInterface::originalImageInput(const asn1SccFrame& data)
{
    inOriginalImage = data;
}

const asn1SccFrame& ColorConversionInterface::convertedImageOutput() const
{
    return outConvertedImage;
}

}
}

/** @} */
