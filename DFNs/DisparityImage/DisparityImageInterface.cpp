/**
 * @addtogroup DFNs
 * @{
 */

#include "DisparityImageInterface.hpp"

namespace CDFF
{
namespace DFN
{

DisparityImageInterface::DisparityImageInterface()
{
}

DisparityImageInterface::~DisparityImageInterface()
{
}

void DisparityImageInterface::framePairInput(const asn1SccFramePair& data)
{
    inFramePair = data;
}

const asn1SccFrame& DisparityImageInterface::disparityOutput() const
{
    return outDisparity;
}

}
}

/** @} */
