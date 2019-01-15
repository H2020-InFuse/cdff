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
