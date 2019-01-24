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
    asn1SccFramePair_Initialize(&inFramePair);
    asn1SccFrame_Initialize(&outRawDisparity);
}

DisparityImageInterface::~DisparityImageInterface()
{
}

void DisparityImageInterface::framePairInput(const asn1SccFramePair& data)
{
    inFramePair = data;
}

const asn1SccFrame& DisparityImageInterface::rawDisparityOutput() const
{
    return outRawDisparity;
}

}
}

/** @} */
