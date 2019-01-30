/**
 * @addtogroup DFNs
 * @{
 */

#include "StereoDegradationInterface.hpp"

namespace CDFF
{
namespace DFN
{

StereoDegradationInterface::StereoDegradationInterface()
{
    asn1SccFramePair_Initialize(& inOriginalImagePair);
    asn1SccFramePair_Initialize(& outDegradedImagePair);
}

StereoDegradationInterface::~StereoDegradationInterface()
{
}

void StereoDegradationInterface::originalImagePairInput(const asn1SccFramePair& data)
{
    inOriginalImagePair = data;
}

const asn1SccFramePair& StereoDegradationInterface::degradedImagePairOutput() const
{
    return outDegradedImagePair;
}

}
}

/** @} */
