/**
 * @addtogroup DFNs
 * @{
 */

#include "ImagePairDegradationInterface.hpp"

namespace CDFF
{
namespace DFN
{

ImagePairDegradationInterface::ImagePairDegradationInterface()
{
}

ImagePairDegradationInterface::~ImagePairDegradationInterface()
{
}

void ImagePairDegradationInterface::originalImagePairInput(const asn1SccFramePair& data)
{
    inOriginalImagePair = data;
}

const asn1SccFramePair& ImagePairDegradationInterface::degradedImagePairOutput() const
{
    return outDegradedImagePair;
}

}
}

/** @} */
